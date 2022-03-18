#include <iostream>
#include "rosbag_analysis_task/error_detector.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosbag_analysis_task_node");
	ROS_INFO("ROS bag analysis task!");

	ErrorDetector detector;

	detector.init();

	detector.detect();

	ros::shutdown();
	

	return 0;
}