#include "../include/ImageProcessor.h"

using namespace cv;
using namespace std;

ImageProcessor::ImageProcessor()
{
  original_img = imread (IMAGE_DIRECTORY_PATH, IMREAD_COLOR);
}

ImageProcessor::~ImageProcessor()
{

}

void ImageProcessor::getWarpedImage(int offset_x, int offset_y, int offset_x_dst)
{
  //90 80 200
  //Points for image transformation
  Point2f pointsScr[4];
  Point2f pointsDst[4];
  Mat lambda( 2, 4, CV_32FC1 );
  lambda = Mat::zeros( original_img.rows, original_img.cols, original_img.type() ); 

  //Image Transformation
  int x = round(original_img.rows/11.2105) + offset_x;
  pointsScr[0] = Point2f( x, 0+offset_y );
  pointsScr[1] = Point2f( original_img.cols-x, 0+offset_y );
  pointsScr[2] = Point2f( offset_x, original_img.rows-offset_y );
  pointsScr[3] = Point2f( original_img.cols-offset_x, original_img.rows-offset_y ); 

  pointsDst[0] = Point2f( 0, 0 );
  pointsDst[1] = Point2f( original_img.cols, 0 );
  pointsDst[2] = Point2f( offset_x_dst, original_img.rows );
  pointsDst[3] = Point2f( original_img.cols-offset_x_dst, original_img.rows ); 

  lambda = getPerspectiveTransform( pointsScr, pointsDst );
  warpPerspective(original_img,warped_img,lambda,warped_img.size() );
}

void ImageProcessor::getMask()
{
  Mat blur_img;
  GaussianBlur(warped_img, blur_img, Size(11, 11), 0);
  Mat hsv_img;
  cvtColor(blur_img, hsv_img, COLOR_BGR2HSV);
  inRange(hsv_img, Scalar(LOWER_YELLOW_1, LOWER_YELLOW_2, LOWER_YELLOW_3),
    Scalar(UPPER_YELLOW_1, UPPER_YELLOW_2, UPPER_YELLOW_3), mask_img);
}

void ImageProcessor::getResultImage()
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  result_img = warped_img.clone();
  findContours(mask_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  drawContours(result_img, contours, 0, Scalar(255, 255, 0), 3);

  RotatedRect rect = minAreaRect(contours[0]);
  cv::Point2f rect_points[4]; 
  rect.points( rect_points );
  path_angle = rect.angle;
  path_center = rect.center;

  // draw rotated rect
  for(unsigned int j=0; j<4; ++j)
      cv::line(result_img, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 3);
}

void ImageProcessor::printAll() 
{
  stringstream ss;   ss << path_angle; // convert float to string
  circle(result_img, path_center, 5, cv::Scalar(0,255,0)); // draw center
  putText(result_img, ss.str(), path_center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); // print angle
  
  imshow("Original", original_img);
  imshow("Warped", warped_img);
  imshow("Mask", mask_img);
  imshow("Contornos", result_img);

  waitKey(0);
}
