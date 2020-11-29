#include "../include/ImageProcessor.h"
#include <iostream>

int main(void)
{
  ImageProcessor my_processor;
  my_processor.getWarpedImage(90,80,200);
  my_processor.getMask();
  my_processor.getResultImage();
  my_processor.printAll();
}