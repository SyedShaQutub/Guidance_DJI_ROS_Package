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


 using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)


 ros::Publisher image_pubs[10];
 ros::Publisher imu_pub;
 ros::Publisher obstacle_distance_pub;
 ros::Publisher velocity_pub;
 ros::Publisher ultrasonic_pub;
 ros::Publisher motion_pub;

 cv_bridge::CvImage images[10];

 Mat greyscales[10] {Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),
                    Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1)};

 char       key         = 0;
 bool       show_images = 0;
 uint8_t    verbosity = 0;
 DJI_lock   g_lock;
 DJI_event  g_event;

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
    for (int i = 0; i < 5; ++i)
    {
      int j = 2*i;
      if(data->m_greyscale_image_left[i]){
        memcpy(greyscales[j].data, data->m_greyscale_image_left[i], IMAGE_SIZE);
		greyscales[j].copyTo(images[j].image);
        images[j].header.stamp  = ros::Time::now();
        images[j].encoding    = sensor_msgs::image_encodings::MONO8;
        image_pubs[j].publish(images[j].toImageMsg());
      }
      if(data->m_greyscale_image_right[i]){
        memcpy(greyscales[j+1].data, data->m_greyscale_image_right[i], IMAGE_SIZE);
		greyscales[j+1].copyTo(images[j+1].image);
        images[j+1].header.stamp = ros::Time::now();
        images[j+1].encoding = sensor_msgs::image_encodings::MONO8;
        image_pubs[j+1].publish(images[j+1].toImageMsg());
      }
    }
    key = waitKey(1);
  }
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
  /* obstacle distance */
  if ( e_obstacle_distance == data_type && NULL != content )
  {
      obstacle_distance *oa = (obstacle_distance*)content;
      if (verbosity > 1) {
          printf( "frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp );
          printf( "obstacle distance:" );
          for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
          {
              printf( " %f ", 0.01f * oa->distance[i] );
          }
          printf( "\n" );
      }

		// publish obstacle distance
		sensor_msgs::LaserScan g_oa;
		g_oa.ranges.resize(CAMERA_PAIR_NUM);
		g_oa.header.frame_id = "guidance";
		g_oa.header.stamp    = ros::Time::now();
		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
			g_oa.ranges[i] = 0.01f * oa->distance[i];
		obstacle_distance_pub.publish(g_oa);
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
  }
  if(argc>=2 && !strcmp(argv[1], "h")){
    printf("This program publish all the cameras in topics \n from /guidance/cameras/0/left to /guidance/cameras/4/right\n"
           "press 'q' to quit.");
    return 0;
  }

  /* initialize ros */
  ros::init(argc, argv, "GuidanceCameras");
  ros::NodeHandle my_node;

  image_pubs[0] = my_node.advertise<sensor_msgs::Image>("/guidance/0/left",1);
  image_pubs[1] = my_node.advertise<sensor_msgs::Image>("/guidance/0/right",1);
  image_pubs[2] = my_node.advertise<sensor_msgs::Image>("/guidance/1/left",1);
  image_pubs[3] = my_node.advertise<sensor_msgs::Image>("/guidance/1/right",1);
  image_pubs[4] = my_node.advertise<sensor_msgs::Image>("/guidance/2/left",1);
  image_pubs[5] = my_node.advertise<sensor_msgs::Image>("/guidance/2/right",1);
  image_pubs[6] = my_node.advertise<sensor_msgs::Image>("/guidance/3/left",1);
  image_pubs[7] = my_node.advertise<sensor_msgs::Image>("/guidance/3/right",1);
  image_pubs[8] = my_node.advertise<sensor_msgs::Image>("/guidance/4/left",1);
  image_pubs[9] = my_node.advertise<sensor_msgs::Image>("/guidance/4/right",1);

  imu_pub      = my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
  velocity_pub = my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
  motion_pub   = my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/motion",1);


  images[0].header.frame_id = "guidanceDown";
  images[1].header.frame_id = "guidanceDown";
  images[2].header.frame_id = "guidanceFront";
  images[3].header.frame_id = "guidanceFront";
  images[4].header.frame_id = "guidanceRight";
  images[5].header.frame_id = "guidanceRight";
  images[6].header.frame_id = "guidanceRear";
  images[7].header.frame_id = "guidanceRear";
  images[8].header.frame_id = "guidanceLeft";
  images[9].header.frame_id = "guidanceLeft";

  /* initialize guidance */
  reset_config();
  int err_code = init_transfer();
  RETURN_IF_ERR(err_code);

  int online_status[CAMERA_PAIR_NUM];
  err_code = get_online_status(online_status);
  RETURN_IF_ERR(err_code);
  std::cout<<"Sensor online status: ";
  for (int i=0; i<CAMERA_PAIR_NUM; i++)
    std::cout<<online_status[i]<<" ";
  std::cout<<std::endl;

  // get cali param
  stereo_cali cali[CAMERA_PAIR_NUM];
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  std::cout<<"cu\tcv\tfocal\tbaseline\n";
  for (int i=0; i<5; i++)
  {
    std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
  }

  /* select data */
  err_code = select_greyscale_image(e_vbus1, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus1, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus2, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus2, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus3, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus3, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus4, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus4, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus5, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus5, false);
  RETURN_IF_ERR(err_code);

  select_imu();
  select_velocity();
  select_motion();
  select_obstacle_distance();

  /* start data transfer */
  err_code = set_sdk_event_handler(my_callback);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);

  std::cout << "start_transfer" << std::endl;

  while (ros::ok())
  {
    g_event.wait_event();
    ros::spinOnce();
    if (key > 0){
     if (key == 'q'){
        err_code = stop_transfer();
		RETURN_IF_ERR(err_code);
		reset_config();
      break;
    }
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
