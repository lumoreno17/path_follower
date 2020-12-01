#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSO_H

#include <string.h>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#define LOWER_YELLOW_1 10
#define LOWER_YELLOW_2 50
#define LOWER_YELLOW_3 0

#define UPPER_YELLOW_1 50
#define UPPER_YELLOW_2 255
#define UPPER_YELLOW_3 255

using namespace cv;
using namespace std;

class ImageProcessor
{
public:
  ImageProcessor(std::string img_path);
  ~ImageProcessor();
  Mat original_img, warped_img, mask_img, result_img;
  int width, height;
  float path_angle;
  Point2f path_center;

  void getWarpedImage(int offset_x, int offset_y, int offset_x_dst);
  void getMask();
  void getResultImage();
  void printAll();
};

#endif