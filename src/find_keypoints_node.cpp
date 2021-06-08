/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>

#include <opencv2/features2d.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <stright_moving/KeyPointsVec.h>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "find_keypoints_node.h"

uint frames = 0;
bool has_keypoints = false;
stright_moving::KeyPointsVec keypoints_msg;
cv::Ptr<cv::Feature2D> orb;

int find_keypoints(cv::Mat& image, int32_t sqns); //HACK sind die Globals  keypoints_msg good?

// receive camera frame
void image_readCB(const sensor_msgs::Image &message_holder)
{
	uint width = 0, height = 0;
	has_keypoints = false;
	// Bild konvertieren
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		// assign opencv image pointer
		cv_ptr = cv_bridge::toCvCopy(message_holder, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (width == 0 & height == 0) // /HACK bei dem ersem aufruf
	{

		// width = message_holder.width;
		// height = message_holder.height;
		// //--- INITIALIZE VIDEOWRITER
		
		// int codec = cv::VideoWriter::fourcc('X', '2', '6', '4'); // select desired codec (must be available at runtime)
		// double fps = 30.0;										 // framerate of the created video stream
		// std::string filename = "./video_fu.mp4";					 // name of the output video file
		// bool isColor = false;
		// writer.open(filename, cv::CAP_GSTREAMER, fps, cv::Size(width, height), isColor);
		// // check if we succeeded
		// if (!writer.isOpened())
		// {
		// 	ROS_ERROR("Could not open the output video file for write\n");
		// 	return; //TODO exception hinzufuegen
		// }
	}

	int32_t sqns = message_holder.header.seq;
	frames++;

	/// find keypoints
	find_keypoints(cv_ptr->image, sqns);

	if (sqns % 100 == 0)
		ROS_INFO("image sequence is about: %d", sqns);
	return;
}

// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_keypoints_node");

	ros::NodeHandle nh;
	//	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */

	// private_nh.param<std::string>("device", camera_device, camera_device);
	
	ROS_INFO("starting keypoints calculation");

	ros::Subscriber image_subscriber = nh.subscribe("jetbot_camera/raw", 1, image_readCB);

	//publish a twist_value computed by this controller;
	ros::Publisher keypoints_publisher = nh.advertise<stright_moving::KeyPointsVec>("image_keypoints", 1);

	keypoints_msg.img_keypoints.resize(MAX_POINTS);
	orb = cv::ORB::create(MAX_POINTS,1.2,8,31,0,2,cv::ORB::HARRIS_SCORE,31,20);

	/*
	 * start publishing video frames
	 */
	while (ros::ok()) // && frames < 50)
	{
		if( has_keypoints ) //keypoints_publisher.getNumSubscribers() > 0 ) // OPTI Leistung pptimieren
			keypoints_publisher.publish(keypoints_msg); // 

		ros::spinOnce();
	}

	return 0;
}
