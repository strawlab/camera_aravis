/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include <arv.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <glib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisConfig.h>


//#define TUNING	// Allows tuning the gains for the timestamp controller.  Publishes output on topic /dt, and receives gains on params /kp, /ki, /kd


//#define MIN(a,b)		(((a)<(b)) ? (a) : (b))
//#define MAX(a,b)		(((a)>(b)) ? (a) : (b))
#define CLIP(x,lo,hi)	MIN(MAX((lo),(x)),(hi))
#define THROW_ERROR(m) throw std::string((m))

typedef camera_aravis::CameraAravisConfig Config;

ArvStream      *CreateStream(void);
static gboolean emit_software_trigger_callback (void *abstract_data);


// Global variables -------------------
struct global_s
{
	gboolean 								bCancel;
	image_transport::CameraPublisher 		publisher;
	camera_info_manager::CameraInfoManager *pCameraInfoManager;
	sensor_msgs::CameraInfo 				camerainfo;
	gint 									width, height; // buffer->width and buffer->height not working, so I used a global.
	Config 									config;
	Config 									configMin;
	Config 									configMax;
	int                                     idsrcTrigger;
	
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
#ifdef TUNING			
	ros::Publisher 							*ppubInt64;
#endif

} global;


typedef struct 
{
    GMainLoop *main_loop;
    int        buffer_count;
} ApplicationData;
// ------------------------------------


// Conversions from integers to Arv types.
const char 			*szTriggersourceFromInt[] 		= {"Software", "Line1", "Line2"};
ArvAcquisitionMode 	 arvAcquisitionModeFromInt[] 	= {ARV_ACQUISITION_MODE_CONTINUOUS, ARV_ACQUISITION_MODE_SINGLE_FRAME};
ArvAuto 			 arvAutoFromInt[]				= {ARV_AUTO_OFF, ARV_AUTO_ONCE, ARV_AUTO_CONTINUOUS};
const char 			*szBufferStatusFromInt[] 		= {
														"ARV_BUFFER_STATUS_SUCCESS",
														"ARV_BUFFER_STATUS_CLEARED",
														"ARV_BUFFER_STATUS_TIMEOUT",
														"ARV_BUFFER_STATUS_MISSING_PACKETS",
														"ARV_BUFFER_STATUS_WRONG_PACKET_ID",
														"ARV_BUFFER_STATUS_SIZE_MISMATCH",
														"ARV_BUFFER_STATUS_FILLING",
														"ARV_BUFFER_STATUS_ABORTED"
														};


static void set_cancel (int signal)
{
    global.bCancel = TRUE;
}

ArvStream *CreateStream(void)
{
	gboolean arv_option_auto_socket_buffer = FALSE;
	gboolean arv_option_no_packet_resend = FALSE;
	unsigned int arv_option_packet_timeout = 40; // milliseconds
	unsigned int arv_option_frame_retention = 200;

	
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
	std::string		stGuid;

	gint payload = arv_camera_get_payload (global.pArvcamera);
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
    
    if (*pWidth > 0)
    	*pWidth = CLIP(*pWidth,  global.widthRoiMin,  global.widthRoiMax - *pX);
    else
    	*pWidth = global.widthRoiMax - *pX;
    
    if (*pHeight > 0)
    	*pHeight  = CLIP(*pHeight, global.heightRoiMin, global.heightRoiMax - *pY);
    else
    	*pHeight = global.heightRoiMax - *pY;

}

void ros_reconfigure_callback(Config &config, uint32_t level)
{
	ros::Duration   dur(2.0);
    int             changedFramerate;
    int             changedAutoexposure;
    int             changedAutogain;
    int             changedExposure;
    int             changedGain;
    int             changedAcquisitionMode;
    int             changedTriggersource;
    int             changedSoftwarerate;
//    int             changedRoi;
    int             changedFrameid;
    
    const char     *szTriggersource;
    
    
    std::string tf_prefix = tf::getPrefixParam(*global.pNode);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    
    szTriggersource = szTriggersourceFromInt[config.triggersource];
    if (config.frame_id == "")
        config.frame_id = "camera";

    
    // Find what's changed.
    changedFramerate    = (global.config.framerate != config.framerate);
    changedAutoexposure = (global.config.autoexposure != config.autoexposure);
    changedAutogain     = (global.config.autogain != config.autogain);
    changedExposure     = (global.config.exposure != config.exposure);
    changedGain         = (global.config.gain != config.gain);
    changedAcquisitionMode = (global.config.acquisitionmode != config.acquisitionmode);
    changedTriggersource= (global.config.triggersource != config.triggersource);
    changedSoftwarerate  = (global.config.softwarerate != config.softwarerate);
//    changedRoi          = (global.config.xRoi != config.xRoi) // Aravis has trouble changing ROI on-the-fly.
//    						|| (global.config.yRoi != config.yRoi) 
//    						|| (global.config.widthRoi != config.widthRoi)
//    						|| (global.config.heightRoi != config.heightRoi);
    changedFrameid      = (global.config.frame_id != config.frame_id);

    // Limit params to legal values.
    config.framerate  = CLIP(config.framerate, global.configMin.framerate, global.configMax.framerate);
    config.exposure   = CLIP(config.exposure,  global.configMin.exposure,  global.configMax.exposure);
    config.gain       = CLIP(config.gain,      global.configMin.gain,      global.configMax.gain);
//    ClipRoi (&config.xRoi, &config.yRoi, &config.widthRoi, &config.heightRoi);
    config.frame_id   = tf::resolve(tf_prefix, config.frame_id);
    if (changedExposure || ((changedFramerate 
    		                 || changedAutogain || changedGain || changedFrameid 
    		                 || changedAcquisitionMode || changedTriggersource || changedSoftwarerate) && arvAutoFromInt[config.autoexposure]==ARV_AUTO_ONCE)) 
    	config.autoexposure = (int)ARV_AUTO_OFF;
    if (changedGain || ((changedFramerate || changedAutoexposure || changedExposure || changedFrameid 
    		             || changedAcquisitionMode || changedTriggersource || changedSoftwarerate) && arvAutoFromInt[config.autogain]==ARV_AUTO_ONCE)) 
    	config.autogain = (int)ARV_AUTO_OFF;

    
    // Set params into the camera.
    if (changedExposure)
    {
    	ROS_INFO ("Set exposure time = %f", config.exposure);
    	arv_camera_set_exposure_time(global.pArvcamera, config.exposure);
    }
    if (changedGain)
    {
    	ROS_INFO ("Set gain = %f", config.gain);
    	arv_camera_set_gain(global.pArvcamera, config.gain);
    }
    if (changedAutoexposure)
    {
    	ROS_INFO ("Set autoexposure = %s", arv_auto_to_string(arvAutoFromInt[config.autoexposure]));
    	arv_camera_set_exposure_time_auto(global.pArvcamera, arvAutoFromInt[config.autoexposure]);
    	dur.sleep();
        config.exposure = arv_camera_get_exposure_time (global.pArvcamera);
        config.autoexposure = (int)ARV_AUTO_OFF;
    }
    if (changedAutogain)
    {
    	ROS_INFO ("Set autogain = %s", arv_auto_to_string(arvAutoFromInt[config.autogain]));
    	arv_camera_set_gain_auto(global.pArvcamera, arvAutoFromInt[config.autogain]);
    	dur.sleep();
        config.gain = arv_camera_get_gain (global.pArvcamera);
    	config.autogain = (int)ARV_AUTO_OFF;
    }
    if (changedAcquisitionMode)
    {
    	ROS_INFO ("Set acquisition mode = %s", arv_acquisition_mode_to_string(arvAcquisitionModeFromInt[config.acquisitionmode]));
    	arv_camera_set_acquisition_mode(global.pArvcamera, arvAcquisitionModeFromInt[config.acquisitionmode]);
    }
    if (changedFramerate)
    {
    	ROS_INFO ("Set framerate = %f", config.framerate);
    	arv_camera_set_frame_rate(global.pArvcamera, config.framerate);
    }
    if (changedTriggersource)
    {
    	ROS_INFO ("Set triggersource = %s", szTriggersource);
    	arv_camera_set_trigger_source (global.pArvcamera, szTriggersource);
    	arv_camera_set_trigger (global.pArvcamera, szTriggersource);
    }
    if (changedTriggersource || changedSoftwarerate)
    {
    	if (!g_strcmp0(szTriggersource,"Software"))
    	{
        	ROS_INFO ("Set softwarerate = %f", 1000.0/ceil(1000.0 / config.softwarerate));
    		// Turn on software timer callback.
    		if (global.idsrcTrigger)
    			g_source_remove(global.idsrcTrigger);
    			
    		global.idsrcTrigger = g_timeout_add ((guint)ceil(1000.0 / config.softwarerate), emit_software_trigger_callback, global.pArvcamera);
    	}
    	else
    	{
    		// Turn off software timer callback.
    		if (global.idsrcTrigger)
    			g_source_remove(global.idsrcTrigger);
    			
    	}
    }
//    if (changedRoi)
//    {
//    	arv_camera_stop_acquisition (global.pArvcamera);
//    	arv_camera_set_region(global.pArvcamera, config.xRoi, config.yRoi, config.widthRoi, config.heightRoi);
//    	//arv_camera_get_region(global.pArvcamera, &config.xRoi, &config.yRoi, &config.widthRoi, &config.heightRoi);
//    	arv_camera_start_acquisition (global.pArvcamera);
//    }


    global.config = config;
}


static void new_buffer_cb (ArvStream *pStream, ApplicationData *pApplicationdata)
{
	static uint64_t  cm = 0L;	// Camera time prev
	uint64_t  		 cn = 0L;	// Camera time now

#ifdef TUNING			
	static uint64_t  rm = 0L;	// ROS time prev
#endif
	uint64_t  		 rn = 0L;	// ROS time now

	static uint64_t	 tm = 0L;	// Calculated image time prev
	uint64_t		 tn = 0L;	// Calculated image time now
		
	static int64_t   em = 0L;	// Error prev.
	int64_t  		 en = 0L;	// Error now between calculated image time and ROS time.
	int64_t  		 de = 0L;	// derivative.
	int64_t  		 ie = 0L;	// integral.
	int64_t			 u = 0L;	// Output of controller.
	
	int64_t			 kp1 = 0L;		// Fractional gains in integer form.
	int64_t			 kp2 = 1024L;
	int64_t			 kd1 = 0L;
	int64_t			 kd2 = 1024L;
	int64_t			 ki1 = -1L;		// A gentle pull toward zero.
	int64_t			 ki2 = 1024L;

	static uint32_t	 iFrame = 0;	// Frame counter.
    
	ArvBuffer		*pBuffer;

	
#ifdef TUNING			
	std_msgs::Int64  msgInt64;
	int 			 kp = 0;
	int 			 kd = 0;
	int 			 ki = 0;
    
	if (ros::param::has("/kp"))
	{
		ros::param::get("/kp", kp);
		kp1 = kp;
	}
	
	if (ros::param::has("/kd"))
	{
		ros::param::get("/kd", kd);
		kd1 = kd;
	}
	
	if (ros::param::has("/ki"))
	{
		ros::param::get("/ki", ki);
		ki1 = ki;
	}
#endif
	
    pBuffer = arv_stream_try_pop_buffer (pStream);
    if (pBuffer != NULL) 
    {
        if (pBuffer->status == ARV_BUFFER_STATUS_SUCCESS) 
        {
			sensor_msgs::Image msg;
			
        	pApplicationdata->buffer_count++;
			std::vector<uint8_t> this_data(pBuffer->size);
			memcpy(&this_data[0], pBuffer->data, pBuffer->size);


			// Camera/ROS Timestamp coordination.
			cn				= (uint64_t)pBuffer->timestamp_ns;				// Camera now
			rn	 			= ros::Time::now().toNSec();					// ROS now
			
			if (iFrame < 10)
			{
				cm = cn;
				tm  = rn;
			}
			
			// Control the error between the computed image timestamp and the ROS timestamp.
			en = (int64_t)tm + (int64_t)cn - (int64_t)cm - (int64_t)rn; // i.e. tn-rn, but calced from prior values.
			de = en-em;
			ie += en;
			u = kp1*(en/kp2) + ki1*(ie/ki2) + kd1*(de/kd2);  // kp<0, ki<0, kd>0
			
			// Compute the new timestamp.
			tn = (uint64_t)((int64_t)tm + (int64_t)cn-(int64_t)cm + u);

#ifdef TUNING			
			ROS_WARN("en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld", en, ie, de, kp1*(en/kp2), ki1*(ie/ki2), kd1*(de/kd2), u);
			ROS_WARN("cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld", cn, rn, cn-cm, rn-rm, (int64_t)tn-(int64_t)tm, tn-rn);
			msgInt64.data = tn-rn; //cn-cm+tn-tm; //
			global.ppubInt64->publish(msgInt64);
			rm = rn;
#endif
			
			// Save prior values.
			cm = cn;
			tm = tn;
			em = en;
			
			// Construct the image message.
			msg.header.stamp.fromNSec(tn);
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
        else
        	ROS_WARN ("Frame error: %s", szBufferStatusFromInt[pBuffer->status]);
        	
        arv_stream_push_buffer (pStream, pBuffer);
        iFrame++;
    }
}

static void control_lost_cb (ArvGvDevice *gv_device)
{
    ROS_WARN ("Control lost.");

    global.bCancel = TRUE;
}

static gboolean emit_software_trigger_callback (void *pArvcamera)
{
    arv_camera_software_trigger ((ArvCamera*)pArvcamera);

    return TRUE;
}

static gboolean periodic_task_cb (void *abstract_data)
{
    ApplicationData *pData = (ApplicationData*)abstract_data;

    //  ROS_INFO ("Frame rate = %d Hz", pData->buffer_count);
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
    char   *pszGuid = NULL;
    char    szGuid[512];
    int		nInterfaces = 0;
    int		nDevices = 0;
    int 	i=0;
    int     widthSensor;
    int     heightSensor;
    
    
    global.bCancel = FALSE;
    global.config = global.config.__getDefault__();
    global.idsrcTrigger = 0;

    ros::init(argc, argv, "camnode");
    if (ros::this_node::getNamespace() == "/") 
    {
        ROS_WARN("[camnode] Started in the global namespace! This is probably wrong. Start camnode "
                 "in the camera namespace.\nExample command-line usage:\n"
                 "\t$ ROS_NAMESPACE=my_camera rosrun camera_aravis camnode\n");
    }


    g_type_init ();

    // Print out some useful info.
    ROS_INFO ("Attached cameras:");
    arv_update_device_list();
    nInterfaces = arv_get_n_interfaces();
    ROS_INFO ("# Interfaces: %d", nInterfaces);

    nDevices = arv_get_n_devices();
    ROS_INFO ("# Devices: %d", nDevices);
    for (i=0; i<nDevices; i++)
    	ROS_INFO ("Device%d: %s", i, arv_get_device_id(i));
    
    
    if (nDevices>0)
    {
		// Get the camera guid from either the command-line or as a parameter.
    	if (argc==2)
    	{
    		strcpy(szGuid, argv[1]);
    		pszGuid = szGuid;
    	}
    	else
    		if (ros::param::has("guid"))
    		{
    			std::string		stGuid;
    			
    			ros::param::get("guid", stGuid);
    			strcpy (szGuid, stGuid.c_str());
        		pszGuid = szGuid;
    		}
    		else
    			pszGuid = NULL;
    	
    	
    	// Open the camera, and set it up.
		global.pArvcamera = arv_camera_new(pszGuid);
		if (global.pArvcamera != NULL) 
		{
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
			
			// Get ROI from parameter server.
			if (ros::param::has("roi/x"))
				ros::param::get("roi/x", global.xRoi);
			else
				global.xRoi = 0;
	
			if (ros::param::has("roi/y"))
				ros::param::get("roi/y", global.yRoi);
			else
				global.yRoi = 0;
			
			if (ros::param::has("roi/width"))
				ros::param::get("roi/width", global.widthRoi);
			else
				global.widthRoi = 0;
	
			if (ros::param::has("roi/height"))
				ros::param::get("roi/height", global.heightRoi);
			else
				global.heightRoi = 0;
			
			ClipRoi (&global.xRoi, &global.yRoi, &global.widthRoi, &global.heightRoi);
	
			// Initial camera settings.
			arv_camera_set_exposure_time_auto(global.pArvcamera, arvAutoFromInt[global.config.autoexposure]);
			arv_camera_set_gain_auto(global.pArvcamera, arvAutoFromInt[global.config.autogain]);
			arv_camera_set_exposure_time(global.pArvcamera, global.config.exposure);
			arv_camera_set_gain(global.pArvcamera, global.config.gain);
			arv_camera_set_region (global.pArvcamera, global.xRoi, global.yRoi, global.widthRoi, global.heightRoi);
			arv_camera_set_binning (global.pArvcamera, arv_option_horizontal_binning, arv_option_vertical_binning);
			arv_camera_set_acquisition_mode (global.pArvcamera, arvAcquisitionModeFromInt[global.config.acquisitionmode]);
			arv_camera_set_frame_rate(global.pArvcamera, global.config.framerate);

#ifdef TUNING			
			ros::Publisher pubInt64 = global.pNode->advertise<std_msgs::Int64>("dt", 100);
			global.ppubInt64 = &pubInt64;
#endif
    	
			// Start the camerainfo manager.
			global.pCameraInfoManager = new camera_info_manager::CameraInfoManager(*global.pNode, arv_camera_get_device_id(global.pArvcamera));
		
			// Start the dynamic_reconfigure server.
			dynamic_reconfigure::Server<Config> srv;
			dynamic_reconfigure::Server<Config>::CallbackType fnCallback;
			fnCallback = boost::bind(&ros_reconfigure_callback, _1, _2);
			srv.setCallback(fnCallback);
			ros::Duration(2.0).sleep();
		
			// Get parameter current values.
			arv_camera_get_region (global.pArvcamera, &global.xRoi, &global.yRoi, &global.widthRoi, &global.heightRoi);
			arv_camera_get_binning (global.pArvcamera, &dx, &dy);
			global.config.exposure  = arv_camera_get_exposure_time (global.pArvcamera);
			global.config.gain      = arv_camera_get_gain (global.pArvcamera);
			global.config.framerate = arv_camera_get_frame_rate (global.pArvcamera);
			global.pszPixelformat   = g_string_ascii_down(g_string_new(arv_camera_get_pixel_format_as_string(global.pArvcamera)))->str;
	
			
			
			// Print information.
			ROS_INFO ("    Using Camera Configuration:");
			ROS_INFO ("    ---------------------------");
			ROS_INFO ("    Vendor name          = %s", arv_camera_get_vendor_name (global.pArvcamera));
			ROS_INFO ("    Model name           = %s", arv_camera_get_model_name (global.pArvcamera));
			ROS_INFO ("    Device id            = %s", arv_camera_get_device_id (global.pArvcamera));
			ROS_INFO ("    Sensor width         = %d", widthSensor); 
			ROS_INFO ("    Sensor height        = %d", heightSensor);
			ROS_INFO ("    ROI x,y,w,h          = %d, %d, %d, %d", global.xRoi, global.yRoi, global.widthRoi, global.heightRoi);
			ROS_INFO ("    Horizontal binning   = %d", dx);
			ROS_INFO ("    Vertical binning     = %d", dy);
			ROS_INFO ("    Pixel format         = %s", global.pszPixelformat);
			ROS_INFO ("    Acquisition Mode     = %s", arv_acquisition_mode_to_string(arv_camera_get_acquisition_mode(global.pArvcamera)));
			ROS_INFO ("    Framerate            = %g hz", global.config.framerate);
			ROS_INFO ("    Trigger Source       = %s", arv_camera_get_trigger_source(global.pArvcamera));
			ROS_INFO ("    Exposure             = %g us in range [%g,%g]", global.config.exposure, global.configMin.exposure, global.configMax.exposure);
			ROS_INFO ("    Gain                 = %g %% in range [%g,%g]", global.config.gain, global.configMin.gain, global.configMax.gain);
			ROS_INFO ("    Can set Framerate:     %s", arv_camera_is_frame_rate_available(global.pArvcamera) ? "True" : "False");
			ROS_INFO ("    Can set AutoExposure:  %s", arv_camera_is_exposure_auto_available(global.pArvcamera) ? "True" : "False");
			ROS_INFO ("    Can set AutoGain:      %s", arv_camera_is_gain_auto_available(global.pArvcamera) ? "True" : "False");
			ROS_INFO ("    Can set Exposure:      %s", arv_camera_is_exposure_time_available(global.pArvcamera) ? "True" : "False");
			ROS_INFO ("    Can set Gain:          %s", arv_camera_is_gain_available(global.pArvcamera) ? "True" : "False");
			
			ArvStream *pStream = CreateStream();
		
			// topic is "image_raw", with queue size of 1
			// image transport interfaces
			image_transport::ImageTransport *transport = new image_transport::ImageTransport(*global.pNode);
			global.publisher = transport->advertiseCamera("image_raw", 1);
		
			arv_camera_stop_acquisition (global.pArvcamera);
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
		
			if (global.idsrcTrigger)
				g_source_remove(global.idsrcTrigger);
		
			signal (SIGINT, pSigintHandlerOld);
		
			g_main_loop_unref (applicationdata.main_loop);
		
			{
				guint64 n_completed_buffers;
				guint64 n_failures;
				guint64 n_underruns;
		
				arv_stream_get_statistics (pStream, &n_completed_buffers, &n_failures, &n_underruns);
	
				ROS_INFO ("Completed buffers = %Lu", (unsigned long long) n_completed_buffers);
				ROS_INFO ("Failures          = %Lu", (unsigned long long) n_failures);
				ROS_INFO ("Underruns         = %Lu", (unsigned long long) n_underruns);
			}
			arv_camera_stop_acquisition (global.pArvcamera);
		
			g_object_unref (pStream);
		} // if (global.pArvcamera != NULL)
		else
			ROS_WARN ("Could not open camera: %s", pszGuid ? pszGuid : "NULL");
    }
    else
    	ROS_WARN ("No cameras detected.");
    
    return 0;
}
