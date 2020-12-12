#ifndef PATH_FOLLOWER_CONTROLLER_H
#define PATH_FOLLOWER_CONTROLLER_H

#include <string.h>
#include <iostream>
#include <vector>

using namespace std;

class PathFollowerController
{
public:
  PathFollowerController();
  ~PathFollowerController();
  float m_max_speed, m_min_speed, m_w_right, m_w_left, m_w_robot, m_v_robot, m_PID;
  float m_derivator, m_integrator, m_max_integrator, m_min_integrator, m_Kp, m_Kd, m_Ki;
  float V;

  std::vector<float> speed_control(float distance_diff, float angular_diff, float K);
  std::vector<float> calcWheelsSpeed(float V, float W);
  float PIDController(float error);
  float saturate(float current_value, float max_value, float min_value);
  int checkDirection(float angle);
};

#endif