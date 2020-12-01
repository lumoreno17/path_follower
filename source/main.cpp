#include "../include/ImageProcessor.h"
#include <iostream>

#define IMAGE_DIRECTORY_PATH1 "../images/linha_pespec.jpg"
#define IMAGE_DIRECTORY_PATH2 "../images/linha-amarela2.jpg"
#define IMAGE_DIRECTORY_PATH3 "../images/linha-amarela3.jpg"
#define IMAGE_DIRECTORY_PATH4 "../images/linha-amarela4.jpg"

int main(void)
{
  ImageProcessor my_processor(IMAGE_DIRECTORY_PATH3);
  //my_processor.getWarpedImage(90,80,200); //PATH1
  //my_processor.getWarpedImage(60,50,150); //PATH2
  my_processor.getWarpedImage(30,20,30); //PATH2
  //my_processor.getWarpedImage(100,90,100); //PATH4
  my_processor.getMask();
  my_processor.getResultImage();
  my_processor.printAll();
}