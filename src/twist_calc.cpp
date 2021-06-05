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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>

#include "my_image_converter.h"

// Include opencv2
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>

  // Include CvBridge, Image Transport, Image msg
  #include <image_transport/image_transport.h>
  #include <cv_bridge/cv_bridge.h>
  

// bool imageConverter::Convert( sensor_msgs::Image& msg, imageFormat format )
// {
// 	return Convert(msg, format, ImageGPU());
// }

typedef uchar3 PixelType;

// Convert
bool Convert( sensor_msgs::Image& msg, imageFormat format, PixelType* imageGPU )
{

	/*
	if( !mInputCPU || !imageGPU || mWidth == 0 || mHeight == 0 || mSizeInput == 0 || mSizeOutput == 0 )
		return false;
	
	// perform colorspace conversion into the desired encoding
	// in this direction, we reverse use of input/output pointers
	if( CUDA_FAILED(cudaConvertColor(imageGPU, InternalFormat, mInputGPU, format, mWidth, mHeight)) )
	{
		ROS_ERROR("failed to convert %ux%u image (from %s to %s) with CUDA", mWidth, mHeight, imageFormatToStr(InternalFormat), imageFormatToStr(format));
		return false;
	}

	// calculate size of the msg
	const size_t msg_size = imageFormatSize(format, mWidth, mHeight);

	// allocate msg storage
	msg.data.resize(msg_size);

	// copy the converted image into the msg
	memcpy(msg.data.data(), mInputCPU, msg_size);

	// populate metadata
	msg.width  = mWidth;
	msg.height = mHeight;
	msg.step   = (mWidth * imageFormatDepth(format)) / 8;

	msg.encoding     = imageFormatToEncoding(format);
	msg.is_bigendian = false;
*/	
	return true;
}

// aquire and publish camera frame
void image_readCB(const sensor_msgs::Image& message_holder)
{
	// Bild konvertieren
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(message_holder, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	cv::Point text_point(30,30);

	// Draw a black background behind text
 	   cv::rectangle(cv_ptr->image, text_point, text_point + cv::Point(100,20) , CV_RGB(0,0,0), cv::LineTypes::FILLED);

	// Convert( message_holder,  format, &imageGPU );

	// Differenz zu gespeichertem Bild auswerten
	// Bild spechern zu spaeterem Vergleich
	// Twist ausgeben

	int32_t sqns =  message_holder.header.seq;

	if (sqns%50 == 0 )
		ROS_INFO("image sequence is: %d",sqns); 
	return;
}

// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "twist_calc");
 
	ros::NodeHandle nh;
//	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */
	
	// private_nh.param<std::string>("device", camera_device, camera_device);
	
	ROS_INFO("starting twist calculation");

	std_msgs::Float64 twist;

	ros::Subscriber image_subscriber = nh.subscribe("jetbot_camera/raw", 1, image_readCB); 
 
	//publish a force command computed by this controller; 
    ros::Publisher twist_publisher = nh.advertise<std_msgs::Float64>("twist_value", 1);


	/*
	 * start publishing video frames
	 */
	while( ros::ok() )
	{
		//if( raw_pub->getNumSubscribers() > 0 )
			twist.data = 3.61;
			twist_publisher.publish(twist);

		ros::spinOnce();
	}

	return 0;
}

