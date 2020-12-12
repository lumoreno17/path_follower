#include "../include/ImageProcessor.h"

using namespace cv;
using namespace std;

ImageProcessor::ImageProcessor()
{
}

ImageProcessor::~ImageProcessor()
{
}

void ImageProcessor::getWarpedImage(cv::Mat input_img, int offset_x, int offset_y, int offset_x_dst)
{
  //90 80 200
  //Points for image transformation
  Point2f pointsScr[4];
  Point2f pointsDst[4];
  Mat lambda( 2, 4, CV_32FC1 );
  lambda = Mat::zeros( input_img.rows, input_img.cols, input_img.type() ); 

  //Image Transformation
  int x = round(input_img.rows/11.2105) + offset_x;
  pointsScr[0] = Point2f( x, 0+offset_y );
  pointsScr[1] = Point2f( input_img.cols-x, 0+offset_y );
  pointsScr[2] = Point2f( offset_x, input_img.rows-offset_y );
  pointsScr[3] = Point2f( input_img.cols-offset_x, input_img.rows-offset_y ); 

  pointsDst[0] = Point2f( 0, 0 );
  pointsDst[1] = Point2f( input_img.cols, 0 );
  pointsDst[2] = Point2f( offset_x_dst, input_img.rows );
  pointsDst[3] = Point2f( input_img.cols-offset_x_dst, input_img.rows ); 

  lambda = getPerspectiveTransform( pointsScr, pointsDst );
  warpPerspective(input_img,warped_img,lambda,warped_img.size() );
}

void ImageProcessor::getMask(cv::Mat input_img)
{
  Mat blur_img;
  GaussianBlur(input_img, blur_img, Size(17, 17), 0);
  Mat hsv_img;
  cvtColor(blur_img, hsv_img, COLOR_BGR2HSV);
  inRange(hsv_img, Scalar(LOWER_YELLOW_1, LOWER_YELLOW_2, LOWER_YELLOW_3),
    Scalar(UPPER_YELLOW_1, UPPER_YELLOW_2, UPPER_YELLOW_3), mask_img);

  erode(mask_img,mask_img, getStructuringElement(MORPH_RECT, Size(3,3)));
}

bool ImageProcessor::getResultImage(cv::Mat input_img)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  result_img = input_img.clone();
  findContours(mask_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  if(contours.size() < 1)
  {
    std::cout << "[ERROR] - Can not process image {Contours size: " << contours.size() << "}" << std::endl;
    return false;
  }
  
  drawContours(result_img, contours, 0, Scalar(255, 255, 0), 3);
  
  RotatedRect rect = minAreaRect(contours[0]);
  cv::Point2f rect_points[4]; 
  rect.points( rect_points );
  path_angle = rect.angle;
  path_center = rect.center;
  width = result_img.cols;
  
  std::vector<float> y_values(4,0.0f);
  std::vector<float> x_values(4,0.0f);
  std::vector<std::vector<float>> controller_points;

  // draw rotated rect
  for(unsigned int j=0; j<4; ++j)
  {
    cv::line(result_img, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 3);
    y_values[j] = rect_points[j].y;
    x_values[j] = rect_points[j].x;
  }
      
  if(rect.size.width < rect.size.height)
    path_angle = -path_angle;
  else
    path_angle = -(path_angle + 90);

  stringstream ss;   ss << path_angle; // convert float to string
  circle(result_img, path_center, 5, cv::Scalar(10,255,100),5); // draw center
  
  controller_points = sortPoints(y_values, x_values);

  auto x = controller_points[1][0] + (controller_points[1][1] - controller_points[1][0])/2;
  auto y = controller_points[0][0] + (controller_points[0][1] - controller_points[0][0])/2; 
  cv::Point new_point;
  new_point.x = x;
  new_point.y = y;
  circle(result_img, new_point, 5, cv::Scalar(255,0,255),5); // draw center

  putText(result_img, ss.str(), path_center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,100,255)); // print angle
  dist_diff = int(width/2) - new_point.x;
  return true;
}

void ImageProcessor::getOutput(int &dist_diff_out, float &path_angle_out)
{
  dist_diff_out = dist_diff;
  path_angle_out = path_angle;
  //std::cout << "[DEBUG] dist_diff: " << dist_diff << "  path_angle: " << path_angle << std::endl;
}

void ImageProcessor::printAll() 
{
  std::cout << "Image x center: " << int(width/2) << "\nRect x center: " << path_center.x << "\n Pixel Dist: " << dist_diff << std::endl;
  std::cout << "Angle: " << path_angle << std::endl;
  imshow("Original", original_img);
  imshow("Warped", warped_img);
  imshow("Mask", mask_img);
  imshow("Contornos", result_img);

  waitKey(0);
}

bool ImageProcessor::processImage(cv::Mat image, bool warped, cv::Mat &processed_img)
{
  original_img = image;
  //std::cout << "Rows: " << original_img.rows << " Colums: " << original_img.cols << std::endl;
  getWarpedImage(original_img,20,20,60);
  getMask(warped_img);
  bool ret = getResultImage(warped_img);
  if(ret)
  {
    processed_img = result_img;
    return true;
  }
  else
  {
    std::cout << "[ERROR] Could not process image, returning original one: " << std::endl;
    processed_img = image;
    return false;
  }
}

std::vector<std::vector<float>> ImageProcessor::sortPoints(std::vector<float> array_y, std::vector<float> array_x)
{
  std::vector<std::vector<float>> array_out(2,vector<float>(2,0)); 
  vector< pair <float,float> > array_pair; 

  for (size_t i = 0; i < array_y.size(); i++)
    array_pair.push_back(make_pair(array_y[i],array_x[i]));

  sort(array_pair.begin(), array_pair.end());

  for (size_t i = 0; i < 2; i++)
    array_out[0][i] = array_pair[i+2].first;
  for (size_t i = 0; i < 2; i++)
    array_out[1][i] = array_pair[i+2].second;
  
  return array_out;
}