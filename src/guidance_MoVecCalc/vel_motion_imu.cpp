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

ros::Publisher imu_pub;
ros::Publisher velocity_pub;
ros::Publisher motion_pub;

using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
bool            show_images = 0;
uint8_t         verbosity = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
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
    /* imu */
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
        if (verbosity > 1) {
            printf( "frame index: %d, stamp: %d\n", imu_data->frame_index, imu_data->time_stamp );
            printf( "imu: [%f %f %f %f %f %f %f]\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );

        }
    	// publish imu data
		geometry_msgs::TransformStamped g_imu;
		g_imu.header.frame_id = "guidance";
		g_imu.header.stamp    = ros::Time::now();
		g_imu.transform.translation.x = imu_data->acc_x;
		g_imu.transform.translation.y = imu_data->acc_y;
		g_imu.transform.translation.z = imu_data->acc_z;
		g_imu.transform.rotation.w = imu_data->q[0];
		g_imu.transform.rotation.x = imu_data->q[1];
		g_imu.transform.rotation.y = imu_data->q[2];
		g_imu.transform.rotation.z = imu_data->q[3];
		imu_pub.publish(g_imu);
    }
    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
        if (verbosity > 1) {
            printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
            printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
        }

		// publish velocity
		geometry_msgs::Vector3Stamped g_vo;
		g_vo.header.frame_id = "guidance";
		g_vo.header.stamp    = ros::Time::now();
		g_vo.vector.x = 0.001f * vo->vx;
		g_vo.vector.y = 0.001f * vo->vy;
		g_vo.vector.z = 0.001f * vo->vz;
		velocity_pub.publish(g_vo);
    }

    /* motion */
    if ( e_motion == data_type && NULL != content )
    {
    	motion *mo = (motion*)content;
//        if (verbosity > 1) {
//            printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
//            printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
//        }

		// publish velocity
		geometry_msgs::Vector3Stamped g_mo;
		g_mo.header.frame_id = "guidance";
		g_mo.header.stamp    = ros::Time::now();
		g_mo.vector.x = 0.001f * mo->position_in_global_x;
		g_mo.vector.y = 0.001f * mo->position_in_global_y;
		g_mo.vector.z = 0.001f * mo->position_in_global_z;
		motion_pub.publish(g_mo);
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
    ros::init(argc, argv, "GuidanceNode_USonic_Mot_Vel");
    ros::NodeHandle my_node;
    imu_pub  				= my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    velocity_pub  			= my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    motion_pub  			= my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/motion",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

    select_imu();
    select_velocity();
    select_motion();


    // modified by LMT
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);

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
