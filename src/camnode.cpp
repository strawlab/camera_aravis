/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include <arv.h>

#include <iostream>
#include <stdlib.h>

#include <glib.h>
#include <glib/gprintf.h>

#define THROW_ERROR(m) throw std::string((m))

typedef struct {
	GMainLoop *main_loop;
	int buffer_count;
} ApplicationData;

static gboolean cancel = FALSE;

static void
set_cancel (int signal)
{
	cancel = TRUE;
}

static void
new_buffer_cb (ArvStream *stream, ApplicationData *data)
{
	ArvBuffer *buffer;

	buffer = arv_stream_pop_buffer (stream);
	if (buffer != NULL) {
		if (buffer->status == ARV_BUFFER_STATUS_SUCCESS)
			data->buffer_count++;
		/* Image processing here */
		arv_stream_push_buffer (stream, buffer);
	}
}

static void
control_lost_cb (ArvGvDevice *gv_device)
{
	g_printf ("Control lost\n");

	cancel = TRUE;
}

static gboolean
emit_software_trigger (void *abstract_data)
{
	ArvCamera *camera = (ArvCamera*)abstract_data;

	arv_camera_software_trigger (camera);

	return TRUE;
}

static gboolean
periodic_task_cb (void *abstract_data)
{
	ApplicationData *data = (ApplicationData*)abstract_data;

	g_printf ("Frame rate = %d Hz\n", data->buffer_count);
	data->buffer_count = 0;

	if (cancel) {
		g_main_loop_quit (data->main_loop);
		return FALSE;
	}

	return TRUE;
}

int main(int argc, char** argv) {
    ArvCamera *camera;
	char *arv_option_camera_name = NULL;

	g_thread_init (NULL);
	g_type_init ();

    camera = arv_camera_new(arv_option_camera_name);
	if (camera == NULL) {
		THROW_ERROR("could not open camera");
	}

	std::cout << "opened camera" << std::endl;

	{
		int arv_option_width = -1;
		int arv_option_height = -1;
		int arv_option_horizontal_binning = -1;
		int arv_option_vertical_binning = -1;
		double arv_option_exposure_time_us = -1;
		int arv_option_gain = -1;
		gint x, y, width, height;
		gint dx, dy;
		double exposure;
		int gain;

		arv_camera_set_region (camera, 0, 0, arv_option_width, arv_option_height);
		arv_camera_set_binning (camera, arv_option_horizontal_binning, arv_option_vertical_binning);
		arv_camera_set_exposure_time (camera, arv_option_exposure_time_us);
		arv_camera_set_gain (camera, arv_option_gain);

		arv_camera_get_region (camera, &x, &y, &width, &height);
		arv_camera_get_binning (camera, &dx, &dy);
		exposure = arv_camera_get_exposure_time (camera);
		gain = arv_camera_get_gain (camera);

		g_printf ("vendor name         = %s\n", arv_camera_get_vendor_name (camera));
		g_printf ("model name          = %s\n", arv_camera_get_model_name (camera));
		g_printf ("device id           = %s\n", arv_camera_get_device_id (camera));
		g_printf ("image width         = %d\n", width);
		g_printf ("image height        = %d\n", height);
		g_printf ("horizontal binning  = %d\n", dx);
		g_printf ("vertical binning    = %d\n", dy);
		g_printf ("exposure            = %g µs\n", exposure);
		g_printf ("gain                = %d dB\n", gain);
	}

	gint payload = arv_camera_get_payload (camera);

	ArvStream *stream = arv_camera_create_stream (camera, NULL, NULL);
	if (stream == NULL) {
		THROW_ERROR("could not open stream");
	}

	if (!ARV_IS_GV_STREAM (stream)) {
		THROW_ERROR("stream is not GV_STREAM");
	}

	{

		gboolean arv_option_auto_socket_buffer = FALSE;
		gboolean arv_option_no_packet_resend = FALSE;
		unsigned int arv_option_packet_timeout = 20;
		unsigned int arv_option_frame_retention = 100;

		if (arv_option_auto_socket_buffer)
			g_object_set (stream,
						  "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
						  "socket-buffer-size", 0,
						  NULL);
		if (arv_option_no_packet_resend)
			g_object_set (stream,
						  "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER,
						  NULL);
		g_object_set (stream,
					  "packet-timeout", (unsigned) arv_option_packet_timeout * 1000,
					  "frame-retention", (unsigned) arv_option_frame_retention * 1000,
					  NULL);
	}

	for (int i = 0; i < 50; i++)
		arv_stream_push_buffer (stream, arv_buffer_new (payload, NULL));

	arv_camera_set_acquisition_mode (camera, ARV_ACQUISITION_MODE_CONTINUOUS);

	guint software_trigger_source = 0;
	{
		char *arv_option_trigger = NULL;
		double arv_option_software_trigger = -1;
		double arv_option_frequency = -1.0;

		if (arv_option_frequency > 0.0)
			arv_camera_set_frame_rate (camera, arv_option_frequency);

		if (arv_option_trigger != NULL)
			arv_camera_set_trigger (camera, arv_option_trigger);

		if (arv_option_software_trigger > 0.0) {
			arv_camera_set_trigger (camera, "Software");
			software_trigger_source = g_timeout_add ((double) (0.5 + 1000.0 /
															   arv_option_software_trigger),
													 emit_software_trigger, camera);
		}

	}


	arv_camera_start_acquisition (camera);
	ApplicationData data;

	g_signal_connect (stream, "new-buffer", G_CALLBACK (new_buffer_cb), &data);
	arv_stream_set_emit_signals (stream, TRUE);

	g_signal_connect (arv_camera_get_device (camera), "control-lost",
					  G_CALLBACK (control_lost_cb), NULL);

	g_timeout_add_seconds (1, periodic_task_cb, &data);

	data.main_loop = g_main_loop_new (NULL, FALSE);

	void (*old_sigint_handler)(int);
	old_sigint_handler = signal (SIGINT, set_cancel);

	g_main_loop_run (data.main_loop);

	if (software_trigger_source > 0)
		g_source_remove (software_trigger_source);

	signal (SIGINT, old_sigint_handler);

	g_main_loop_unref (data.main_loop);

	{
		guint64 n_completed_buffers;
		guint64 n_failures;
		guint64 n_underruns;

		arv_stream_get_statistics (stream, &n_completed_buffers, &n_failures, &n_underruns);

		g_printf ("Completed buffers = %Lu\n", (unsigned long long) n_completed_buffers);
		g_printf ("Failures          = %Lu\n", (unsigned long long) n_failures);
		g_printf ("Underruns         = %Lu\n", (unsigned long long) n_underruns);
	}
	arv_camera_stop_acquisition (camera);

	g_object_unref (stream);

    return 0;
}
