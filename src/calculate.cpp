/**
  @file videowriter_basic.cpp
  @brief A very basic sample for using VideoWriter and VideoCapture
  @author PkLab.net
  @date Aug 24, 2016
*/

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int calculate(cv::Mat& image, int32_t sqns)
{
   
	char image_name[512];

	sprintf(image_name,"./stright_moving/images/img-%08d.jpg", sqns);

     //  writer.write(cv_ptr->image);
	   cv::imwrite(image_name, image); //HACK cv bridge has opencv 3

    return 0;
}
