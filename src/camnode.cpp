/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include <arv.h>

#include <iostream>
#include <stdlib.h>

#include <glib.h>
#include <glib/gprintf.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisConfig.h>

#define THROW_ERROR(m) throw std::string((m))
typedef camera_aravis::CameraAravisConfig Config;

// Global variables -------------------
struct
{
	gboolean 								bCancel;
	image_transport::CameraPublisher 		publisher;
	camera_info_manager::CameraInfoManager *pCameraInfoManager;
	sensor_msgs::CameraInfo 				camerainfo;
	gint 									width, height; // buffer->width and buffer->height not working, so I used a global.
	Config 									config;
	ros::NodeHandle 					   *pNode;
	ArvCamera 							   *pArvcamera;
} global;


typedef struct 
{
    GMainLoop *main_loop;
    int        buffer_count;
} ApplicationData;
// ------------------------------------



static void set_cancel (int signal)
{
    global.bCancel = TRUE;
}

void ros_reconfigure_callback(Config &configNew, uint32_t level)
{
    std::string tf_prefix = tf::getPrefixParam(*global.pNode);

    if (configNew.frame_id == "")
        configNew.frame_id = "camera";

    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    configNew.frame_id = tf::resolve(tf_prefix, configNew.frame_id);

    global.config = configNew;                // save new parameters

    arv_camera_set_exposure_time(global.pArvcamera, global.config.exposure);
    arv_camera_set_gain(global.pArvcamera, global.config.gain);
    arv_camera_set_frame_rate(global.pArvcamera, global.config.framerate);

}

static void new_buffer_cb (ArvStream *pStream, ApplicationData *pApplicationdata)
{
    ArvBuffer *pBuffer;

    pBuffer = arv_stream_pop_buffer (pStream);
    if (pBuffer != NULL) 
    {
        if (pBuffer->status == ARV_BUFFER_STATUS_SUCCESS) 
        {
            pApplicationdata->buffer_count++;
            int step = global.width; // XXX how to check this?
            std::vector<uint8_t> this_data(pBuffer->size);
            memcpy(&this_data[0], pBuffer->data, pBuffer->size);

            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time::now(); // host timestamps (else pBuffer->timestamp_ns)
            msg.header.seq = pBuffer->frame_id;
            msg.header.frame_id = global.config.frame_id;
            msg.height = global.height;
            msg.width = global.width;
            msg.encoding = "mono8";  // XXX fixme
            msg.step = step;
            msg.data = this_data;

            // get current CameraInfo data
            global.camerainfo = global.pCameraInfoManager->getCameraInfo();
            global.camerainfo.header.stamp = msg.header.stamp;
            global.camerainfo.header.seq = msg.header.seq;
            global.camerainfo.header.frame_id = msg.header.frame_id;
            global.camerainfo.height = global.height;
            global.camerainfo.width = global.width;

            global.publisher.publish(msg, global.camerainfo);
        }
        arv_stream_push_buffer (pStream, pBuffer);
    }
}

static void control_lost_cb (ArvGvDevice *gv_device)
{
    g_printf ("Control lost\n");

    global.bCancel = TRUE;
}

static gboolean emit_software_trigger (void *abstract_data)
{
    ArvCamera *pArvcamera = (ArvCamera*)abstract_data;

    arv_camera_software_trigger (pArvcamera);

    return TRUE;
}

static gboolean periodic_task_cb (void *abstract_data)
{
    ApplicationData *pData = (ApplicationData*)abstract_data;

    //  g_printf ("Frame rate = %d Hz\n", pData->buffer_count);
    pData->buffer_count = 0;

    if (global.bCancel) {
        g_main_loop_quit (pData->main_loop);
        return FALSE;
    }

    ros::spinOnce();

    return TRUE;
}

int main(int argc, char** argv) 
{
    char   *pszCamera = NULL;
    int		nInterfaces = 0;
    int		nDevices = 0;
    int 	i=0;
    
    global.bCancel = FALSE;
    global.config = global.config.__getDefault__();

    ros::init(argc, argv, "camnode");
    if (ros::this_node::getNamespace() == "/") 
    {
        ROS_WARN("[camnode] Started in the global namespace! This is probably wrong. Start camnode "
                 "in the camera namespace.\nExample command-line usage:\n"
                 "\t$ ROS_NAMESPACE=my_camera rosrun camera_aravis camnode\n");
    }

    //g_thread_init (NULL);
    g_type_init ();

    // Print out some useful info.
    ROS_WARN ("Attached cameras:");
    arv_update_device_list();
    nInterfaces = arv_get_n_interfaces();
    ROS_WARN ("# Interfaces: %d", nInterfaces);

    nDevices = arv_get_n_devices();
    ROS_WARN ("# Devices: %d", nDevices);
    for (i=0; i<nDevices; i++)
    	ROS_WARN ("Device%d: %s", i, arv_get_device_id(i));
    
    
    if (nDevices>0)
    {
		global.pArvcamera = arv_camera_new(pszCamera);
		if (global.pArvcamera == NULL) 
			ROS_WARN ("Could not open camera.");
	
		global.pNode = new ros::NodeHandle();
	
		int arv_option_width = -1;
		int arv_option_height = -1;
		int arv_option_horizontal_binning = -1;
		int arv_option_vertical_binning = -1;
		//double arv_option_exposure_time_us = -1;
		//int arv_option_gain = -1;
		gint x, y;
		gint dx, dy;
		double exposure, exposureMin, exposureMax;
		double gain, gainMin, gainMax;

		// Manual exposure mode
		arv_camera_set_exposure_time_auto(global.pArvcamera, ARV_AUTO_OFF);
		arv_camera_set_gain_auto(global.pArvcamera, ARV_AUTO_OFF);

		arv_camera_set_region (global.pArvcamera, 0, 0, arv_option_width, arv_option_height);
		arv_camera_set_binning (global.pArvcamera, arv_option_horizontal_binning, arv_option_vertical_binning);

		arv_camera_set_exposure_time(global.pArvcamera, global.config.exposure);
		arv_camera_set_gain(global.pArvcamera, global.config.gain);

		arv_camera_get_region (global.pArvcamera, &x, &y, &global.width, &global.height);
		arv_camera_get_binning (global.pArvcamera, &dx, &dy);
		exposure = arv_camera_get_exposure_time (global.pArvcamera);
		arv_camera_get_exposure_time_bounds(global.pArvcamera, &exposureMin, &exposureMax);
		gain = arv_camera_get_gain (global.pArvcamera);
		arv_camera_get_gain_bounds(global.pArvcamera, &gainMin, &gainMax);

		g_printf ("vendor name         = %s\n", arv_camera_get_vendor_name (global.pArvcamera));
		g_printf ("model name          = %s\n", arv_camera_get_model_name (global.pArvcamera));
		g_printf ("device id           = %s\n", arv_camera_get_device_id (global.pArvcamera));
		g_printf ("image width         = %d\n", global.width);
		g_printf ("image height        = %d\n", global.height);
		g_printf ("horizontal binning  = %d\n", dx);
		g_printf ("vertical binning    = %d\n", dy);
		g_printf ("pixel format        = %s\n", arv_camera_get_pixel_format_as_string(global.pArvcamera));
		g_printf ("exposure            = %g µs in range [%g,%g]\n", exposure, exposureMin, exposureMax);
		g_printf ("gain                = %g %% in range [%g,%g]\n", gain, gainMin, gainMax);
		g_printf ("is_frame_rate_available = %s\n", arv_camera_is_frame_rate_available(global.pArvcamera) ? "True" : "False");
		
		std::string ros_camera_name = arv_camera_get_device_id(global.pArvcamera);
		global.pCameraInfoManager = new camera_info_manager::CameraInfoManager(*global.pNode, ros_camera_name);
	
		dynamic_reconfigure::Server<Config> srv;
		dynamic_reconfigure::Server<Config>::CallbackType fnCallback;
		fnCallback = boost::bind(&ros_reconfigure_callback, _1, _2);
		srv.setCallback(fnCallback);
	
		gint payload = arv_camera_get_payload (global.pArvcamera);
	
		ArvStream *pStream = arv_camera_create_stream (global.pArvcamera, NULL, NULL);
		if (pStream == NULL) 
		{
			THROW_ERROR("could not open stream");
		}
	
		if (!ARV_IS_GV_STREAM (pStream)) 
		{
			THROW_ERROR("stream is not GV_STREAM");
		}
	
		{
	
			gboolean arv_option_auto_socket_buffer = FALSE;
			gboolean arv_option_no_packet_resend = FALSE;
			unsigned int arv_option_packet_timeout = 20;
			unsigned int arv_option_frame_retention = 100;
	
			if (arv_option_auto_socket_buffer)
				g_object_set (pStream,
							  "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
							  "socket-buffer-size", 0,
							  NULL);
			if (arv_option_no_packet_resend)
				g_object_set (pStream,
							  "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER,
							  NULL);
			g_object_set (pStream,
						  "packet-timeout", (unsigned) arv_option_packet_timeout * 1000,
						  "frame-retention", (unsigned) arv_option_frame_retention * 1000,
						  NULL);
		}
	
		for (int i = 0; i < 50; i++)
			arv_stream_push_buffer (pStream, arv_buffer_new (payload, NULL));
	
		// topic is "image_raw", with queue size of 1
		// image transport interfaces
		image_transport::ImageTransport *transport = new image_transport::ImageTransport(*global.pNode);
		global.publisher = transport->advertiseCamera("image_raw", 1);
	
		arv_camera_set_acquisition_mode (global.pArvcamera, ARV_ACQUISITION_MODE_CONTINUOUS);
	
		guint software_trigger_source = 0;
		{
			char *arv_option_trigger = NULL;
			double arv_option_software_trigger = -1;
			double arv_option_frequency = 1000.0;
	
			if (arv_option_frequency > 0.0)
				arv_camera_set_frame_rate (global.pArvcamera, arv_option_frequency);
	
			if (arv_option_trigger != NULL)
				arv_camera_set_trigger (global.pArvcamera, arv_option_trigger);
	
			if (arv_option_software_trigger > 0.0) 
			{
				arv_camera_set_trigger (global.pArvcamera, "Software");
				software_trigger_source = g_timeout_add ((double) (0.5 + 1000.0 / arv_option_software_trigger),
														 emit_software_trigger, global.pArvcamera);
			}
	
		}
	
	
		arv_camera_start_acquisition (global.pArvcamera);
		ApplicationData applicationdata;
		applicationdata.buffer_count=0;
		applicationdata.main_loop = 0;
	
		g_signal_connect (pStream, "new-buffer", G_CALLBACK (new_buffer_cb), &applicationdata);
		arv_stream_set_emit_signals (pStream, TRUE);
	
		g_signal_connect (arv_camera_get_device (global.pArvcamera), "control-lost",
						  G_CALLBACK (control_lost_cb), NULL);
	
		g_timeout_add_seconds (1, periodic_task_cb, &applicationdata);
	
		applicationdata.main_loop = g_main_loop_new (NULL, FALSE);
	
		void (*pSigintHandlerOld)(int);
		pSigintHandlerOld = signal (SIGINT, set_cancel);
	
		g_main_loop_run (applicationdata.main_loop);
	
		if (software_trigger_source > 0)
			g_source_remove (software_trigger_source);
	
		signal (SIGINT, pSigintHandlerOld);
	
		g_main_loop_unref (applicationdata.main_loop);
	
		{
			guint64 n_completed_buffers;
			guint64 n_failures;
			guint64 n_underruns;
	
			arv_stream_get_statistics (pStream, &n_completed_buffers, &n_failures, &n_underruns);
	
			g_printf ("Completed buffers = %Lu\n", (unsigned long long) n_completed_buffers);
			g_printf ("Failures          = %Lu\n", (unsigned long long) n_failures);
			g_printf ("Underruns         = %Lu\n", (unsigned long long) n_underruns);
		}
		arv_camera_stop_acquisition (global.pArvcamera);
	
		g_object_unref (pStream);
    }
    else
    	ROS_WARN ("No cameras detected.");
    
    return 0;
}
