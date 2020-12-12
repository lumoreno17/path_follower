#include <stdio.h>
#include <stdlib.h>
extern "C"
{
#include "../include/extApi.h"
}
#include "../include/ImageProcessor.h"
#include "../include/PathFollowerController.h"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  int clientID = simxStart((simxChar *)"127.0.0.1", 19997, true, true, 5000, 5);
  if (clientID != -1)
  {
    simxSynchronous(clientID, true);
    simxStartSimulation(clientID, simx_opmode_oneshot_wait);
    printf("Connected to remote API server\n");
    simxAddStatusbarMessage(clientID, "Hello CoppeliaSim!", simx_opmode_oneshot);
  }
  else
  {
    printf("Não foi possível conectar-se ao Coppelia Sim");
    simxFinish(clientID);
    return 0;
  }
  int left_motor, right_motor, robot, camera;
  float *position, *orientation;
  int resolution[2];
  simxUChar* image;
  ImageProcessor img_processor;
  PathFollowerController controller;
  //simxGetObjectPosition(clientID, robot, -1, position, simx_opmode_blocking);
  //simxGetObjectOrientation(clientID, robot, -1, orientation, simx_opmode_blocking);
  simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &left_motor, simx_opmode_blocking);
  simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &right_motor, simx_opmode_blocking);
  simxGetObjectHandle(clientID, "Pioneer_p3dx", &robot, simx_opmode_blocking);
  simxGetObjectHandle(clientID,"Vision_sensor", &camera, simx_opmode_blocking);
  simxGetVisionSensorImage(clientID,camera,resolution,&image,0,simx_opmode_streaming);
  
  simxSetJointTargetVelocity(clientID, left_motor, 0.0f, simx_opmode_blocking);
  simxSetJointTargetVelocity(clientID, right_motor, 0.0f, simx_opmode_blocking);
  int iteracoes = 0;
  while (simxGetConnectionId(clientID)!=-1 && iteracoes < 10e3)
  {
    int ret = simxGetVisionSensorImage(clientID,camera,resolution,&image,0,simx_opmode_buffer);
    iteracoes++;
    //std::this_thread::sleep_for (std::chrono::seconds(1));
    if (ret != simx_return_ok) {
      #ifdef DEBUG
        std::cout << "[SIMULATION ERROR] - interacao " << iteracoes << std::endl;
      #endif
			continue;
		}
    cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
		//The image data read back is flipped vertically, the problem should be that the direction of the vertical coordinate axis of cvMat and v-rep is opposite, and the flip is normal
		cv::flip(channel, channel, 0);
                                 //The rgb channel is distributed when the image data read back, and cvMat defaults to bgr
		cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);
    cv::Mat processed_img;
    bool sts = img_processor.processImage(channel, true, processed_img);
    int dist;
    float angle;
    img_processor.getOutput(dist, angle);
    #ifdef DEBUG
      std::cout << "[DEBUG] dist_diff: " << dist << "  path_angle: " << angle << std::endl;
    #endif
    /*Controller*/
    std::vector<float> Ww = controller.speed_control(float(dist), angle, 1);
    #ifdef DEBUG
      std::cout << "[DEBUG] W_right: " << Ww[0] << "  W_left:: " << Ww[1] << std::endl;
    #endif
    simxSetJointTargetVelocity(clientID, left_motor, Ww[1], simx_opmode_blocking);
    simxSetJointTargetVelocity(clientID, right_motor, Ww[0], simx_opmode_blocking);

		//cv::imshow( "original img",channel);
    cv::imshow( "Processed img",processed_img);
    
		cv::waitKey(10);
    simxSynchronousTrigger(clientID);
  }

  simxSetJointTargetVelocity(clientID, left_motor, 0.0f, simx_opmode_blocking);
  simxSetJointTargetVelocity(clientID, right_motor, 0.0f, simx_opmode_blocking);

  int pingTime;
  simxGetPingTime(clientID, &pingTime);

  // Now close the connection to CoppeliaSim:
  simxStopSimulation(clientID, simx_opmode_oneshot_wait);
  simxFinish(clientID);
  return (0);
}