/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include <arv.h>

#include <iostream>
#include <stdlib.h>

#include <glib.h>
#include <glib/gprintf.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisConfig.h>


//#define MIN(a,b)		(((a)<(b)) ? (a) : (b))
//#define MAX(a,b)		(((a)>(b)) ? (a) : (b))
#define CLIP(x,lo,hi)	MIN(MAX((lo),(x)),(hi))

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
	Config 									configMin;
	Config 									configMax;
	
	int                                     xRoi;
	int                                     yRoi;
	int                                     widthRoi;
	int                                     heightRoi;

	int                                     xRoiMin;
	int                                     yRoiMin;
	int                                     widthRoiMin;
	int                                     heightRoiMin;

	int                                     xRoiMax;
	int                                     yRoiMax;
	int                                     widthRoiMax;
	int                                     heightRoiMax;
	
	const char                             *pszPixelformat;
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

ArvStream *CreateStream(void)
{
	gboolean arv_option_auto_socket_buffer = FALSE;
	gboolean arv_option_no_packet_resend = FALSE;
	unsigned int arv_option_packet_timeout = 20;
	unsigned int arv_option_frame_retention = 100;

	
	ArvStream *pStream = arv_camera_create_stream (global.pArvcamera, NULL, NULL);

	if (pStream == NULL) 
		ROS_WARN("could not open stream");

	if (!ARV_IS_GV_STREAM (pStream)) 
		ROS_WARN("stream is not GV_STREAM");

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

	gint payload = arv_camera_get_payload (global.pArvcamera);
	g_printf("Payload size %d\n", payload);
	for (int i=0; i<50; i++)
		arv_stream_push_buffer (pStream, arv_buffer_new (payload, NULL));
	
	return pStream;
}	


// ClipRoi()
// Clip the given values such that the rect is a valid image size.
void ClipRoi (int *pX, int *pY, int *pWidth, int *pHeight)
{
    *pX = CLIP(*pX,      global.xRoiMin,      global.xRoiMax);
    *pY = CLIP(*pY,      global.yRoiMin,      global.yRoiMax);
    if (*pWidth != -1)
    	*pWidth = CLIP(*pWidth,  global.widthRoiMin,  global.widthRoiMax - *pX);
    if (*pHeight != -1)
    	*pHeight  = CLIP(*pHeight, global.heightRoiMin, global.heightRoiMax - *pY);
	g_printf("pst: %d, %d, %d, %d\n", *pX, *pY, *pWidth, *pHeight);
}

void ros_reconfigure_callback(Config &config, uint32_t level)
{
	ros::Duration   dur(2.0);
    int             changedFramerate;
    int             changedAutoexposure;
    int             changedAutogain;
    int             changedExposure;
    int             changedGain;
//    int             changedRoi;
    int             changedFrameid;

    
    std::string tf_prefix = tf::getPrefixParam(*global.pNode);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

    if (config.frame_id == "")
        config.frame_id = "camera";

    // Find what's changed.
    changedFramerate    = (global.config.framerate != config.framerate);
    changedAutoexposure = (global.config.autoexposure != config.autoexposure);
    changedAutogain     = (global.config.autogain != config.autogain);
    changedExposure     = (global.config.exposure != config.exposure);
    changedGain         = (global.config.gain != config.gain);
//    changedRoi          = (global.config.xRoi != config.xRoi) // Aravis has trouble changing ROI on-the-fly.
//    						|| (global.config.yRoi != config.yRoi) 
//    						|| (global.config.widthRoi != config.widthRoi)
//    						|| (global.config.heightRoi != config.heightRoi);
    changedFrameid      = (global.config.frame_id != config.frame_id);
    	

    // Limit params to legal values.
    config.framerate  = CLIP(config.framerate, global.configMin.framerate, global.configMax.framerate);
    config.exposure   = CLIP(config.exposure,  global.configMin.exposure,  global.configMax.exposure);
    config.gain       = CLIP(config.gain,      global.configMin.gain,      global.configMax.gain);
//    config.xRoi       = CLIP(config.xRoi,      global.configMin.xRoi,      global.configMax.xRoi);
//    config.yRoi       = CLIP(config.yRoi,      global.configMin.yRoi,      global.configMax.yRoi);
//    if (config.widthRoi != -1)
//    	config.widthRoi   = CLIP(config.widthRoi,  global.configMin.widthRoi,  global.configMax.widthRoi-global.config.xRoi);
//    if (config.heightRoi != -1)
//    	config.heightRoi  = CLIP(config.heightRoi, global.configMin.heightRoi, global.configMax.heightRoi-global.config.yRoi);
    config.frame_id   = tf::resolve(tf_prefix, config.frame_id);
    if (changedExposure || ((changedFramerate || changedAutogain || changedGain || changedFrameid) && (ArvAuto)config.autoexposure==ARV_AUTO_ONCE)) 
    	config.autoexposure = (int)ARV_AUTO_OFF;
    if (changedGain || ((changedFramerate || changedAutoexposure || changedExposure || changedFrameid) && (ArvAuto)config.autogain==ARV_AUTO_ONCE)) 
    	config.autogain = (int)ARV_AUTO_OFF;

    
    // Set params into the camera.
    if (changedFramerate)
    	arv_camera_set_frame_rate(global.pArvcamera, config.framerate);
    if (changedExposure)
    	arv_camera_set_exposure_time(global.pArvcamera, config.exposure);
    if (changedGain)
    	arv_camera_set_gain(global.pArvcamera, config.gain);
//    if (changedRoi)
//    {
//    	arv_camera_stop_acquisition (global.pArvcamera);
//    	arv_camera_set_region(global.pArvcamera, config.xRoi, config.yRoi, config.widthRoi, config.heightRoi);
//    	//arv_camera_get_region(global.pArvcamera, &config.xRoi, &config.yRoi, &config.widthRoi, &config.heightRoi);
//    	arv_camera_start_acquisition (global.pArvcamera);
//    }
    if (changedAutoexposure)
    {
    	arv_camera_set_exposure_time_auto(global.pArvcamera, (ArvAuto)config.autoexposure);
    	dur.sleep();
        config.exposure = arv_camera_get_exposure_time (global.pArvcamera);
    }
    if (changedAutogain)
    {
    	arv_camera_set_gain_auto(global.pArvcamera, (ArvAuto)config.autogain);
    	dur.sleep();
        config.gain = arv_camera_get_gain (global.pArvcamera);
    }

    config.framerate = arv_camera_get_frame_rate (global.pArvcamera);

    global.config = config;
}


static void new_buffer_cb (ArvStream *pStream, ApplicationData *pApplicationdata)
{
    ArvBuffer *pBuffer;

    pBuffer = arv_stream_try_pop_buffer (pStream);

    if (pBuffer != NULL) 
    {
        if (pBuffer->status == ARV_BUFFER_STATUS_SUCCESS) 
        {
        	pApplicationdata->buffer_count++;
			std::vector<uint8_t> this_data(pBuffer->size);
			memcpy(&this_data[0], pBuffer->data, pBuffer->size);

			sensor_msgs::Image msg;
			msg.header.stamp = ros::Time::now(); // host timestamps (else pBuffer->timestamp_ns)
			msg.header.seq = pBuffer->frame_id;
			msg.header.frame_id = global.config.frame_id;
			msg.width = global.widthRoi;
			msg.height = global.heightRoi;
			msg.encoding = global.pszPixelformat;
			msg.step = global.widthRoi;
			msg.data = this_data;

			// get current CameraInfo data
			global.camerainfo = global.pCameraInfoManager->getCameraInfo();
			global.camerainfo.header.stamp = msg.header.stamp;
			global.camerainfo.header.seq = msg.header.seq;
			global.camerainfo.header.frame_id = msg.header.frame_id;
			global.camerainfo.width = global.widthRoi;
			global.camerainfo.height = global.heightRoi;

            global.publisher.publish(msg, global.camerainfo);
        }
        arv_stream_push_buffer (pStream, pBuffer);
    }
}

static void control_lost_cb (ArvGvDevice *gv_device)
{
    g_printf ("Control lost.\n");

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
    int     widthSensor;
    int     heightSensor;
    
    
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
	
		int arv_option_horizontal_binning = -1;
		int arv_option_vertical_binning = -1;
		gint dx, dy;

		// Get parameter bounds.
		arv_camera_get_exposure_time_bounds(global.pArvcamera, &global.configMin.exposure, &global.configMax.exposure);
		arv_camera_get_gain_bounds(global.pArvcamera, &global.configMin.gain, &global.configMax.gain);
		arv_camera_get_sensor_size(global.pArvcamera, &widthSensor, &heightSensor);
		arv_camera_set_region (global.pArvcamera, 0, 0, widthSensor, heightSensor);
		arv_camera_get_width_bounds(global.pArvcamera, &global.widthRoiMin, &global.widthRoiMax);
		arv_camera_get_height_bounds(global.pArvcamera, &global.heightRoiMin, &global.heightRoiMax);
		global.xRoiMin = 0;
		global.xRoiMax = global.widthRoiMax - global.widthRoiMin;
		global.yRoiMin = 0;
		global.yRoiMax = global.heightRoiMax - global.heightRoiMin;
		global.configMin.framerate =    0.0;
		global.configMax.framerate = 1000.0;
		
		ros::param::get("roi/x", global.xRoi);
		ros::param::get("roi/y", global.yRoi);
		ros::param::get("roi/width", global.widthRoi);
		ros::param::get("roi/height", global.heightRoi);
		ClipRoi (&global.xRoi, &global.yRoi, &global.widthRoi, &global.heightRoi);
		
		// -1 or 0 means largest height/width possible.
		if (global.widthRoi<=0)
			global.widthRoi = global.widthRoiMax - global.xRoi;
		if (global.heightRoi<=0)
			global.heightRoi = global.heightRoiMax - global.yRoi;

		
		// Initial camera settings.
		arv_camera_set_exposure_time_auto(global.pArvcamera, (ArvAuto)global.config.autoexposure);
		arv_camera_set_gain_auto(global.pArvcamera, (ArvAuto)global.config.autogain);
		arv_camera_set_exposure_time(global.pArvcamera, global.config.exposure);
		arv_camera_set_gain(global.pArvcamera, global.config.gain);
		//arv_camera_set_region (global.pArvcamera, 0, 0, global.configMax.widthRoi, global.configMax.heightRoi);
		arv_camera_set_region (global.pArvcamera, global.xRoi, global.yRoi, global.widthRoi, global.heightRoi);
		arv_camera_set_binning (global.pArvcamera, arv_option_horizontal_binning, arv_option_vertical_binning);


		// Get parameter current values.
		arv_camera_get_region (global.pArvcamera, &global.xRoi, &global.yRoi, &global.widthRoi, &global.heightRoi);
		arv_camera_get_binning (global.pArvcamera, &dx, &dy);
		global.config.exposure  = arv_camera_get_exposure_time (global.pArvcamera);
		global.config.gain      = arv_camera_get_gain (global.pArvcamera);
		global.config.framerate = arv_camera_get_frame_rate (global.pArvcamera);
		global.pszPixelformat   = g_string_ascii_down(g_string_new(arv_camera_get_pixel_format_as_string(global.pArvcamera)))->str;

		
		
		// Print information.
		g_printf ("Vendor name          = %s\n", arv_camera_get_vendor_name (global.pArvcamera));
		g_printf ("Model name           = %s\n", arv_camera_get_model_name (global.pArvcamera));
		g_printf ("Device id            = %s\n", arv_camera_get_device_id (global.pArvcamera));
		g_printf ("Image width          = %d\n", widthSensor); 
		g_printf ("Image height         = %d\n", heightSensor);
		g_printf ("ROI x,y,w,h          = %d, %d, %d, %d\n", global.xRoi, global.yRoi, global.widthRoi, global.heightRoi);
		g_printf ("Horizontal binning   = %d\n", dx);
		g_printf ("Vertical binning     = %d\n", dy);
		g_printf ("Pixel format         = %s\n", global.pszPixelformat);
		g_printf ("Framerate            = %g fps\n", global.config.framerate);
		g_printf ("Exposure             = %g µs in range [%g,%g]\n", global.config.exposure, global.configMin.exposure, global.configMax.exposure);
		g_printf ("Gain                 = %g %% in range [%g,%g]\n", global.config.gain, global.configMin.gain, global.configMax.gain);
		g_printf ("Can set Framerate:     %s\n", arv_camera_is_frame_rate_available(global.pArvcamera) ? "True" : "False");
		g_printf ("Can set AutoExposure:  %s\n", arv_camera_is_exposure_auto_available(global.pArvcamera) ? "True" : "False");
		g_printf ("Can set AutoGain:      %s\n", arv_camera_is_gain_auto_available(global.pArvcamera) ? "True" : "False");
		g_printf ("Can set Exposure:      %s\n", arv_camera_is_exposure_time_available(global.pArvcamera) ? "True" : "False");
		g_printf ("Can set Gain:          %s\n", arv_camera_is_gain_available(global.pArvcamera) ? "True" : "False");
		
		std::string ros_camera_name = arv_camera_get_device_id(global.pArvcamera);
		global.pCameraInfoManager = new camera_info_manager::CameraInfoManager(*global.pNode, ros_camera_name);
	
		dynamic_reconfigure::Server<Config> srv;
		dynamic_reconfigure::Server<Config>::CallbackType fnCallback;
		fnCallback = boost::bind(&ros_reconfigure_callback, _1, _2);
		srv.setCallback(fnCallback);
	
		ArvStream *pStream = CreateStream();
	
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
