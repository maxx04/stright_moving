

#include <ros/ros.h>

// #include <opencv2/features2d.hpp>

// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>

#include <stright_moving/KeyPointsVec.h>
#include <std_msgs/Float32.h>

// // Include CvBridge, Image Transport, Image msg
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>

#include "find_keypoints_node.h"

std_msgs::Float32 twist_msg;
stright_moving::KeyPointsVec last_keypoints_msg;

// receive camera frame
void keypoints_readCB(const stright_moving::KeyPointsVec &message_holder)
{
	if(message_holder.img_keypoints.size() == 0) return;

	twist_msg.data = 0;

	if(last_keypoints_msg.keypoints_count == 0)
	{
		last_keypoints_msg = message_holder;
		return; // noch kein letzte keypoints waren gespeichert
	} 
	

	for(int i = 0; i < message_holder.keypoints_count; i++)
	{
		// HACK weiterer vorherige punkte beruecksichtigen
		twist_msg.data += last_keypoints_msg.img_keypoints[i].pt.y - 
			message_holder.img_keypoints[i].pt.y;	
	}

	twist_msg.data /= message_holder.keypoints_count;

	last_keypoints_msg = message_holder;

	return;
}

// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_tensors_node");

	ros::NodeHandle nh;
	// ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */

	// private_nh.param<std::string>("device", camera_device, camera_device);
	
	ROS_INFO("starting tensor calculation");

	ros::Subscriber keypoints_subscriber = nh.subscribe("image_keypoints", 1, keypoints_readCB);

	//publish a twist_value computed by this controller;
	ros::Publisher tensor_publisher = nh.advertise<std_msgs::Float32>("twist_angle", 1);

	/*
	 * start publishing video frames
	 */
	while (ros::ok()) // && frames < 10)
	{
//		if( keypoints_publisher.getNumSubscribers() > 0 ) // OPTI Leistung pptimieren
			tensor_publisher.publish(twist_msg); // HACK nur wenn eingangsmessage voll ist (punkte mehr als 0)

		ros::spinOnce();
	}

	return 0;
}
