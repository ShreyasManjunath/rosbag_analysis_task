#ifndef ERROR_DETECTOR_H
#define ERROR_DETECTOR_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <fstream>

class ErrorDetector
{
	public:
		ErrorDetector();
		~ErrorDetector();
		bool init();
		bool detect();

		std::map<std::string, std::string> getErrorCodesMapping() const;
		std::string getBagFileNameAndLocation() const;

	private:
		std::map<std::string, std::string> error_codes;
		std::string bag_file;
		std::vector<std::string> error_codes_v;
		std::vector<std::string> error_codes_comments_v;
		std::vector<std::string> topics;
		std::shared_ptr<rosbag::Bag> bag;
		std::shared_ptr<rosbag::View> view;

		bool present_stop_signal;

	private:
		std::string getMessageTime(rosbag::MessageInstance const& msg) const;
		bool writeIntoFile(std::string& logMessage);
		

};


#endif