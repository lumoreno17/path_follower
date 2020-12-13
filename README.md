# path_follower
This repository simulates a line-follower robot on Coppelia Robotics environment.
This repository has three main files: 
* **ImageProcessor:** A image processor class to draw a rectangle around the path with the same orientation. This element returns the path orientation and the pixel distance between the image center (x direction) and the center of the rectangle down side. Those parameters are the inputs for the controller.
* **PathFollowerController:** Performs a proportional speed control based on the ImageProcessor output parameters
* **main:** Runs the simulation of the **Pioneer** robot on Coppelia environment using the path-follower algorithm developed

## Image Processor:
The picture below shows the original image and processed image. 
A bird-eye view transformation is applied in order to improve the output of the processed image. 
Regarding the processed image, the red rectangle is always oriented to the path. In orange you can see the angle, it varies from -90 to +90 degrees. Zero degrees means the rectangle is in the vertical position. Positive angles means the rectangle is oriented to the left side while negative angles means the rectangle is oriented to the right side.
The pink circle marks the center of the rectangle down side to enable the control based on the distance between the center of view and the path center. 


![Processed Frame](/media/processed_frame.png)

## PathFollowerController
Based on the inputs received, it applies a weighted sum. It means the input parameters are multiplied by a defined configurable weight and the summed. After that a proportional constant is applied and the final value is the calculated angular speed of the robot. With this angular speed and the received linear velocity it calculates the speed that shall be applied in each wheel.

## How to run
After cloning the repository it is necessary to create a a `build` folder and compile.

```shell
$ mkdir build/
$ cd build
$ cmake ../source & make
```

After this, open Coppelia Robotics simulator and load the scene (*<repository_path>/simulation/path_follower.ttt*). Then run:
```shell
$ ./path_follower
```

## Video

![Simulation Record](/media/simulation.gif)

