/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher ultrasonic_pub;


using namespace cv;

namespace camera1
{
    int cAMERA_PAIR_NUM = 1;
}

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
bool            show_images = 0;
uint8_t         verbosity = 0;
e_vbus_index	CAMERA_ID = e_vbus2;
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat				g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);


std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
	const char* s = 0;
	static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value){
		PROCESS_VAL(e_OK);
		PROCESS_VAL(e_load_libusb_err);
		PROCESS_VAL(e_sdk_not_inited);
		PROCESS_VAL(e_disparity_not_allowed);
		PROCESS_VAL(e_image_frequency_not_allowed);
		PROCESS_VAL(e_config_not_ready);
		PROCESS_VAL(e_online_flag_not_ready);
		PROCESS_VAL(e_stereo_cali_not_ready);
		PROCESS_VAL(e_libusb_io_err);
		PROCESS_VAL(e_timeout);
	default:
		strcpy(str, "Unknown error");
		s = str;
		break;
	}
#undef PROCESS_VAL

	return out << s;
}

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {
        image_data* data = (image_data*)content;

		if ( data->m_greyscale_image_left[CAMERA_ID] ){
			memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
			// publish left greyscale image
			cv_bridge::CvImage left_8;
			g_greyscale_image_left.copyTo(left_8.image);
			left_8.header.frame_id  = "guidance/2/left";
			left_8.header.stamp	= ros::Time::now();
			left_8.encoding		= sensor_msgs::image_encodings::MONO8;
			left_image_pub.publish(left_8.toImageMsg());
		}
		if ( data->m_greyscale_image_right[CAMERA_ID] ){
			memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
			// publish right greyscale image
			cv_bridge::CvImage right_8;
			g_greyscale_image_right.copyTo(right_8.image);
			right_8.header.frame_id  = "guidance/2/right";
			right_8.header.stamp	 = ros::Time::now();
			right_8.encoding  	 = sensor_msgs::image_encodings::MONO8;
			right_image_pub.publish(right_8.toImageMsg());
		}
        key = waitKey(1);
    }
    /* obstacle distance */
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
		// publish obstacle distance
		sensor_msgs::LaserScan g_oa;
		g_oa.ranges.resize(camera1::cAMERA_PAIR_NUM);
		g_oa.header.frame_id = "guidance/2/obstacle";
		g_oa.header.stamp    = ros::Time::now();
		for ( int i = 0; i < camera1::cAMERA_PAIR_NUM; ++i )
			g_oa.ranges[i] = 0.01f * oa->distance[i];
		obstacle_distance_pub.publish(g_oa);
	}
    /* ultrasonic */
    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
		// publish ultrasonic data
		sensor_msgs::LaserScan g_ul;
		g_ul.ranges.resize(camera1::cAMERA_PAIR_NUM);
		g_ul.intensities.resize(camera1::cAMERA_PAIR_NUM);
		g_ul.header.frame_id = "guidance/2/ultrasonic";
		g_ul.header.stamp    = ros::Time::now();
		g_ul.ranges[0] = 0.001f*ultrasonic->ultrasonic[CAMERA_ID];
		g_ul.intensities[0] = 1.0 * ultrasonic->reliability[CAMERA_ID];
		ultrasonic_pub.publish(g_ul);
    }

    g_lock.leave();
    g_event.set_event();

    return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

int main(int argc, char** argv)
{
    if (argc < 2) {
        show_images = true;
        verbosity = 2;
    }
	if(argc==2 && !strcmp(argv[1], "h")){
		printf("This is demo program showing data from Guidance.\n\t"
			" 'a','d','w','s','x' to select sensor direction.\n\t"
			" 'j','k' to change the exposure parameters.\n\t"
			" 'm' to switch between AEC and constant exposure modes.\n\t"
			" 'n' to return to default exposure mode and parameters.\n\t"
			" 'q' to quit.");
		return 0;
	}

    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode_Camera1");
    ros::NodeHandle my_node;
    left_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/2/left_image",1);
    right_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/2/right_image",1);
    obstacle_distance_pub	= my_node.advertise<sensor_msgs::LaserScan>("/guidance/2/obstacle_distance",1);
    ultrasonic_pub			= my_node.advertise<sensor_msgs::LaserScan>("/guidance/2/ultrasonic",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

	int online_status[camera1::cAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
    std::cout<<"Sensor online status: ";
    std::cout<<online_status<<" ";
    std::cout<<std::endl;

	// get cali param
	stereo_cali cali;
	err_code = get_stereo_cali(&cali);
	RETURN_IF_ERR(err_code);
    std::cout<<"cu\tcv\tfocal\tbaseline\n";
    std::cout<<cali.cu<<"\t"<<cali.cv<<"\t"<<cali.focal<<"\t"<<cali.baseline<<std::endl;


    /* select data */
    err_code = select_greyscale_image(CAMERA_ID, true);
	RETURN_IF_ERR(err_code);
    err_code = select_greyscale_image(CAMERA_ID, false);
	RETURN_IF_ERR(err_code);
    select_ultrasonic();
    select_obstacle_distance();


    // modified by LMT
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);

	// for setting exposure
	exposure_param para;
	para.m_is_auto_exposure = 1;
	para.m_step = 10;
	para.m_expected_brightness = 120;
    para.m_camera_pair_index = CAMERA_ID;
    set_exposure_param(&para);
	std::cout << "start_transfer" << std::endl;

	while (ros::ok())
	{
		g_event.wait_event();
	    if (key > 0){
	     if (key == 'q'){
	        err_code = stop_transfer();
			RETURN_IF_ERR(err_code);
			reset_config();
	      break;
	     }
         ros::spinOnce();
	    }
	}

	/* release data transfer */
	err_code = stop_transfer();
	RETURN_IF_ERR(err_code);
	//make sure the ack packet from GUIDANCE is received
	sleep(1);
	std::cout << "release_transfer" << std::endl;
	err_code = release_transfer();
	RETURN_IF_ERR(err_code);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
