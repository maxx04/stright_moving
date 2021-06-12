/**
  @file ------------------------videowriter_basic.cpp
  @brief A very basic sample for using VideoWriter and VideoCapture
  @author PkLab.net
  @date Aug 24, 2016
*/

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <stdio.h>

#include <stright_moving/KeyPointsVec.h>

#include "find_keypoints_node.h"

using namespace cv;
using namespace std;

extern bool has_keypoints;
extern stright_moving::KeyPointsVec keypoints_msg;
extern Ptr<Feature2D> orb;

cv::Mat last_image;
vector<Point2f> prev_points; // vorherige punkte

int find_keypoints(cv::Mat &image, int32_t sqns)
{

  // find keypoints

  has_keypoints = false;

  //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  vector<Point2f> key_points;

  //int fast_threshold = 20;
  //bool nonmaxSuppression = true;

  Mat descriptors1, gray;

  if (prev_points.empty()) // fuer ersten anlauf
  {
    cvtColor(image, gray, COLOR_BGR2GRAY); //OPTI

    orb->detectAndCompute(gray, Mat(), keypoints_1, descriptors1);
    KeyPoint::convert(keypoints_1, key_points, vector<int>());

    if (keypoints_1.size() < MIN_KEYPOINTS)
    {
      prev_points.clear();
      return -1;
    }

    prev_points = key_points;
    gray.copyTo(last_image);
    return -1;
  }

  // Terminate Kriterium fuer die LukasKande
  TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

  Size winSize = Size(21, 21); // SubWindow fuer LukasKande

  vector<uchar> status;           // status vom calcOpticalFlowPyrLK
  vector<float> err;              // error vom calcOpticalFlowPyrLK
  vector<Point2f> current_points; // aktuelle punkte

  //	prev_points.clear();

  cvtColor(image, gray, COLOR_BGR2GRAY); //OPTI

  calcOpticalFlowPyrLK(last_image, gray, /*prev*/ prev_points, /*next*/ current_points,
                       status, err, winSize, 5, termcrit, 0, 0.001);

  prev_points = current_points;
  gray.copyTo(last_image);

  // fuelle keypunkte

  stright_moving::KeyPoint kp; //OPTI keypoints direct konvertieren

  int i = 0;
  int count = 0;

  for (Point2f p : current_points)
  {
    if (status[i++] == 1)
    {
      kp.pt.x = p.x;
      kp.pt.y = p.y;

      keypoints_msg.img_keypoints[count++] = kp;

#ifdef DEBUG_KEYPOINTS
      cv::drawMarker(image, p, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
#endif
    }
  }

  keypoints_msg.keypoints_count = count; // HACK count-1 ist nicht notwendig

  keypoints_msg.img_keypoints.resize(count);

#ifdef DEBUG_KEYPOINTS
  char image_name[512];
  sprintf(image_name, "./src/stright_moving/images/img-%08d.jpg", sqns);
  if (!cv::imwrite(image_name, image))
  {
    ROS_WARN("Cannt save to file %s", image_name);
  }
#endif

  has_keypoints = true;

  if (count < MIN_KEYPOINTS)
  {
    has_keypoints = false;
    prev_points.clear(); // um startpoints neu initiieren
    return -1;
  }

  // draw keypoints
  // compare with old keypoints may be 2,3 or 5
  // find moving
  // save keypoints for the next turn
  // calculate twist

  return 0;
}
