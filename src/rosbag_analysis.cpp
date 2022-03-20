/**
 * \file rosbag_analysis.cpp
 * \brief main file
 * */
#include <iostream>

#include "rosbag_analysis_task/error_detector.h"

int main(int argc, char **argv) {
  // ROS inititalize
  ros::init(argc, argv, "rosbag_analysis_task_node");
  ROS_INFO("ROS bag analysis task!");

  // Error detector object
  ErrorDetector detector;

  // Initialization of detector
  detector.init();

  // Detect the error in the rosbag
  detector.detect();

  ros::shutdown();

  return 0;
}