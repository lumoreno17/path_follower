#include "../include/PathFollowerController.h"

using namespace std;

PathFollowerController::PathFollowerController()
{
  m_derivator = 0;
  m_integrator = 0; 
  R = 0.3;
  L = 0.331;
  V = 0.25;
}

PathFollowerController::~PathFollowerController()
{

}

std::vector<float> PathFollowerController::speed_control(float distance_diff, float angle, float K)
{
  float rad_angle  = angle*M_PI/180;
  float w1 = 1e-2;
  float w2 = 1;
  float error_input = w1 * distance_diff + w2*rad_angle;
  float W = K*error_input;
  return calcWheelsSpeed(V, W);
}

std::vector<float> PathFollowerController::calcWheelsSpeed(float V, float W)
{
  std::vector<float> out(2,0.0f);
  out[0] = (V/R) + ((L*W)/(2*R));
  out[1] = (V/R) - ((L*W)/(2*R));

  return out;
}

float PathFollowerController::PIDController(float error)
{
  float p_value = m_Kp*error;
  float d_Value = m_Kd * (error - m_derivator);
  m_derivator = error;
  m_integrator = m_integrator + error;
  m_integrator = saturate(m_integrator, m_max_integrator, m_min_integrator);
  float i_value = m_Ki * m_integrator;

  m_PID = p_value + i_value + d_Value;
  return m_PID;
}

float PathFollowerController::saturate(float current_value, float max_value, float min_value)
{
  if(current_value > max_value)
    current_value = max_value;
  else if(current_value < min_value)
    current_value = min_value;
  else{}  //do nothing

  return current_value;
}

int PathFollowerController::checkDirection(float angle)
{
  /*if(0 < angle < pi/2)
    return 1;
  else
    return -1;*/
  return 1;
}