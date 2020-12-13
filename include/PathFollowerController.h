/**
 * @file PathFollowerController.h
 * @author Felipe Plech (fcplech@gmail.com) / Luciana Borges (lumoreno.borges@gmail.com)
 * @brief Line follower controller (proportional speed control)
 * @version 0.1
 * @date 2020-12-13
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef PATH_FOLLOWER_CONTROLLER_H
#define PATH_FOLLOWER_CONTROLLER_H

#include <string.h>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

class PathFollowerController
{
public:
  /**
   * @brief Construct a new Path Follower Controller object
   * 
   * @param r Wheel radius
   * @param l Distance between wheels
   * @param v Desired linear speed
   */
  PathFollowerController(float r, float l, float v);
  /**
   * @brief Destroy the Path Follower Controller object
   * 
   */
  ~PathFollowerController();
  float V, R, L;
  
  /**
   * @brief Proportional speed control
   * 
   * @param distance_diff Pixel distance between the rectangle centroid and the center of down side
   * @param angular_diff Rectangle orientation
   * @param K Kp constant
   * @return std::vector<float> Wheels speed
   */
  std::vector<float> speed_control(float distance_diff, float angular_diff, float K);
  /**
   * @brief Calculate wheels speed using differencial robot model
   * 
   * @param V Linear velocity
   * @param W Angular velocity
   * @return std::vector<float> Speed
   */
  std::vector<float> calcWheelsSpeed(float V, float W);
};

#endif