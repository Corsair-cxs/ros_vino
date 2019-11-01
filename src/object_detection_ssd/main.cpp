/*
 * File:   main.cpp
 * Author: You Songshan <songshan@lionsbot.com>
 *
 * Created on 30 October 2019
 */

#include "object_detection_ssd.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "object_detection_ssd");

  ObjectDetectionSSD node;

  node.run();

  return 0;
}
