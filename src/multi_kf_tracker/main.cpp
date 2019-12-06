/*
 * File:   main.cpp
 * Author: You Songshan <songshan@lionsbot.com>
 *
 * Created on 3 December 2019
 */

#include "multi_kf_tracker.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "multi_kf_tracker");

  MultiKfTracker node;

  node.run();

  return 0;
}