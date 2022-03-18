#include "rosbag_analysis_task/error_detector.h"
#include <sstream>

ErrorDetector::ErrorDetector()
: present_stop_signal{false}
{
	ros::param::get("/rosbag_analysis_task_node/topics", this->topics);
	ros::param::get("/rosbag_analysis_task_node/bag_file", this->bag_file);
	ros::param::get("/rosbag_analysis_task_node/error_codes", this->error_codes_v);
	ros::param::get("/rosbag_analysis_task_node/error_codes_comments", this->error_codes_comments_v);
}

ErrorDetector::~ErrorDetector()
{
	
}

bool ErrorDetector::init()
{
	for(int i = 0; i < error_codes_v.size(); i++)
	{
		error_codes.insert({this->error_codes_v[i], this->error_codes_comments_v[i]});
	}
	this->bag = std::make_shared<rosbag::Bag>();

	try
	{
		bag->open(bag_file, rosbag::bagmode::Read);
	}catch(rosbag::BagException){
		ROS_ERROR("Error opening file : %s", bag_file.c_str());
		return false;
	}
	this->view = std::make_shared<rosbag::View>();
	this->view->addQuery(*bag, rosbag::TopicQuery(topics));

	return true;
}

bool ErrorDetector::detect()
{
	for(rosbag::MessageInstance const msg: *view)
	{
		std_msgs::Bool::ConstPtr stop_signal = msg.instantiate<std_msgs::Bool>();
		if(stop_signal != NULL)
			this->present_stop_signal = (bool)stop_signal->data;

		std_msgs::String::ConstPtr lts_nav_error =  msg.instantiate<std_msgs::String>();

		if(present_stop_signal && lts_nav_error != NULL)
		{
			auto time_string = getMessageTime(msg);
			auto it = error_codes.find(lts_nav_error->data.c_str());
			std::string reason_for_error;
			if(it != error_codes.end())
				reason_for_error = it->second;
			std::cout << reason_for_error << " @ :" << time_string << std::endl;
		}

	}

	this->bag->close();

	return true;
}

std::string ErrorDetector::getMessageTime(rosbag::MessageInstance const& msg) const
{
	auto time = msg.getTime();
	std::stringstream ss;
	ss << time.sec << "." << time.nsec;
	std::string time_string = ss.str();
	return time_string;
}