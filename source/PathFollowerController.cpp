#include "../include/PathFollowerController.h"

using namespace std;

PathFollowerController::PathFollowerController(float r, float l, float v)
{
  R = r;
  L = l;
  V = v;
}

PathFollowerController::~PathFollowerController()
{

}

std::vector<float> PathFollowerController::speed_control(float distance_diff, float angle, float K)
{
  float rad_angle  = angle*M_PI/180; //Convert from degrees to radians
  float w1 = 4e-2;
  float w2 = 1;
  float error_input = w1 * distance_diff + w2*rad_angle; //Score 
  float W = K*error_input;
  return calcWheelsSpeed(V, W); //Wheels speed
}

std::vector<float> PathFollowerController::calcWheelsSpeed(float V, float W)
{
  std::vector<float> out(2,0.0f);
  out[0] = (V/R) + ((L*W)/(2*R));
  out[1] = (V/R) - ((L*W)/(2*R));

  return out;
}