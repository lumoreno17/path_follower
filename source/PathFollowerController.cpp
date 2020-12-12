#include "../include/PathFollowerController.h"

using namespace std;

PathFollowerController::PathFollowerController()
{
  m_derivator = 0;
  m_integrator = 0; 
}

PathFollowerController::~PathFollowerController()
{

}

std::vector<float> PathFollowerController::speed_control(float distance_diff, float angle, float K)
{
  float w1 = 0.5;
  float w2 = 0.7;
  float error_input = w1 * distance_diff + w2*angle;
  float W = K*error_input;
  return calcWheelsSpeed(V, W);
}

std::vector<float> PathFollowerController::calcWheelsSpeed(float V, float W)
{
  return std::vector<float>();
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