#include "../include/config.hpp"
#include "opencv2/opencv.hpp"
#include "../include/config.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(void)
{
  //Open image
  Mat img_1 = imread (IMAGE_DIRECTORY_PATH, IMREAD_COLOR);
  Mat wrapped_image;
  imshow("Original Image", img_1);

  //Points for image transformation
  Point2f pointsScr[4];
  Point2f pointsDst[4];
  Mat lambda( 2, 4, CV_32FC1 );
  lambda = Mat::zeros( img_1.rows, img_1.cols, img_1.type() ); 

  //Image Transformation
  int offset_x = 90;
  int offset_y = 80;
  int x = round(img_1.rows/11.2105) + offset_x;
  pointsScr[0] = Point2f( x, 0+offset_y );
  pointsScr[1] = Point2f( img_1.cols-x, 0+offset_y );
  pointsScr[2] = Point2f( offset_x, img_1.rows-offset_y );
  pointsScr[3] = Point2f( img_1.cols-offset_x, img_1.rows-offset_y ); 

  pointsDst[0] = Point2f( 0, 0 );
  pointsDst[1] = Point2f( img_1.cols, 0 );
  pointsDst[2] = Point2f( 200, img_1.rows );
  pointsDst[3] = Point2f( img_1.cols-200, img_1.rows ); 

  lambda = getPerspectiveTransform( pointsScr, pointsDst );
  warpPerspective(img_1,wrapped_image,lambda,wrapped_image.size() );

  Mat wrapblurred_img;
  GaussianBlur(wrapped_image, wrapblurred_img, Size(11, 11), 0);

  imshow("Output",wrapblurred_img);

  Mat yellow_mask, hsv_img;

  cvtColor(wrapblurred_img, hsv_img, COLOR_BGR2HSV);
  inRange(hsv_img, Scalar(LOWER_YELLOW_1, LOWER_YELLOW_2, LOWER_YELLOW_3),
   Scalar(UPPER_YELLOW_1, UPPER_YELLOW_2, UPPER_YELLOW_3), yellow_mask);

  imshow("Yellow Mask",yellow_mask);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(yellow_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  drawContours( wrapped_image, contours, 0, Scalar(255, 255, 0), 3);

  RotatedRect rect = minAreaRect(contours[0]);
  cv::Point2f rect_points[4]; 
  rect.points( rect_points );
  float  angle = rect.angle;
  Point2f center = rect.center;

  // draw rotated rect
  for(unsigned int j=0; j<4; ++j)
      cv::line(wrapped_image, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 3);


  stringstream ss;   ss << angle; // convert float to string
  circle(wrapped_image, center, 5, cv::Scalar(0,255,0)); // draw center
  putText(wrapped_image, ss.str(), center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); // print angle

  imshow("Contornos", wrapped_image);

  waitKey(0);
}