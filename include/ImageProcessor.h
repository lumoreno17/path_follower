/**
 * @file ImageProcessor.h
 * @author Felipe Plech (fcplech@gmail.com) / Luciana Borges (lumoreno.borges@gmail.com)
 * @brief Image processor class to apply bird-eye tranformation, draw a retangle with the same orientation
 *  of the path and return the orientation and the pixel distance
 * @version 0.1
 * @date 2020-12-13
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSO_H

#include <string.h>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "simConst.h"

#define LOWER_YELLOW_1 15
#define LOWER_YELLOW_2 75
#define LOWER_YELLOW_3 0

#define UPPER_YELLOW_1 50
#define UPPER_YELLOW_2 255
#define UPPER_YELLOW_3 255

using namespace cv;
using namespace std;

class ImageProcessor
{
public:
  /**
   * @brief Construct a new Image Processor object
   * 
   */
  ImageProcessor();
  /**
   * @brief Destroy the Image Processor object
   * 
   */
  ~ImageProcessor();
  Mat original_img, warped_img, mask_img, result_img;
  int width, height;
  float path_angle;
  int dist_diff;
  Point2f path_center;
  bool warped_flag;
  /**
   * @brief Return the two points coordinates of the rectangle with the maximum Y value
   * 
   * @param array_y Y values
   * @param array_x X values
   * @return std::vector<std::vector<float>> 
   */
  std::vector<std::vector<float>> sortPoints(std::vector<float> array_y, std::vector<float> array_x);
  /**
   * @brief Get the image with bird-eye view transformation
   * 
   * @param image Original image
   * @param offset_x Offset parameter x direction
   * @param offset_y Offset parameter y direction
   * @param offset_x_dst Offset parameter
   */
  void getWarpedImage(cv::Mat image,int offset_x, int offset_y, int offset_x_dst);
  /**
   * @brief Get binary mask image
   * 
   * @param input_img Original image
   */
  void getMask(cv::Mat input_img);
  /**
   * @brief Get the result image with the rectangle in the same orientation of the path
   * 
   * @param input_img Original image
   * @return true Image processing succeed
   * @return false Image processing failed
   */
  bool getResultImage(cv::Mat input_img);
  /**
   * @brief Return image parameters
   * 
   * @param dist_diff_out Pixel distance between the image centroid and the rectangle down edge center (x direction)
   * @param path_angle_out Rectangle orientation
   */
  void getOutput(int &dist_diff_out, float &path_angle_out);
  /**
   * @brief Print all images
   * 
   */
  void printAll();
  /**
   * @brief Process the image and return the result
   * 
   * @param image Original image
   * @param processed_img Processed image
   * @return true 
   * @return false 
   */
  bool processImage(cv::Mat image, cv::Mat &processed_img);
};

#endif