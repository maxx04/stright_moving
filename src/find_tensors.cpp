/**
  @file ------------------------videowriter_basic.cpp
  @brief A very basic sample for using VideoWriter and VideoCapture
  @author PkLab.net
  @date Aug 24, 2016
*/

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>

#include <stright_moving/KeyPointsVec.h>

#include "find_keypoints_node.h"

using namespace cv;
using namespace std;

extern stright_moving::KeyPointsVec keypoints_msg;
extern Ptr<Feature2D> orb;

int find_keypoints(cv::Mat& image, int32_t sqns)
{
// find keypoints

//uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
//	vector<Point2f> key_points;

	//int fast_threshold = 20;
	//bool nonmaxSuppression = true;

	Mat descriptors1;

	orb->detectAndCompute(image, Mat(), keypoints_1, descriptors1);

	// KeyPoint::convert(keypoints_1, key_points, vector<int>());

	//HACK nur notwendige punkte in Zukunft benutzen
	//undistortPoints(key_points, key_points_ud, cameraMatrix, distCoeffs); 

  keypoints_msg.img_keypoints.clear();

	// automatic initialization
  keypoints_msg.keypoints_count = keypoints_1.size();

	// fuelle keypunkte

  int i = 0;
	for (KeyPoint p : keypoints_1)
	{
    keypoints_msg.img_keypoints[i].angle = p.angle;
    keypoints_msg.img_keypoints[i].pt.x = p.pt.x;
    keypoints_msg.img_keypoints[i].pt.y = p.pt.y;
    keypoints_msg.img_keypoints[i].octave = p.octave;
    keypoints_msg.img_keypoints[i].response = p.response;
    keypoints_msg.img_keypoints[i].size = p.size;
    keypoints_msg.img_keypoints[i].class_id = p.class_id;

  #ifdef DEBUG_KEYPOINTS
    cv::drawMarker(image, p.pt,  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
  #endif

    i++;
	}

  // draw keypoints
  // compare with old keypoints may be 2,3 or 5
  // find moving
	// save keypoints for the next turn
	// calculate twist

#ifdef DEBUG_KEYPOINTS
  char image_name[512];
	sprintf(image_name,"./stright_moving/images/img-%08d.jpg", sqns);
	cv::imwrite(image_name, image); 
#endif
    return 0;
}
