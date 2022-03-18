#ifndef ERROR_DETECTOR_H
#define ERROR_DETECTOR_H

#include <ros/ros.h>

class ErrorDetector
{
	public:
		ErrorDetector();
		~ErrorDetector();
		bool init();
		bool detect();

	private:
		ros::NodeHandle nh;

};


#endif