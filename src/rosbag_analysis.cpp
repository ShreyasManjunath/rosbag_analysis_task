#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <ros/ros.h>
#include <memory>
#include <vector>
#include <map>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

std::map<std::string, std::string> error_codes;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosbag_analysis_task_node");
	ROS_INFO("ROS bag analysis task!");

	std::string bag_file;
	std::vector<std::string> error_codes_v;
	std::vector<std::string> error_codes_comments_v;
	ros::param::get("/rosbag_analysis_task_node/error_codes", error_codes_v);
	ros::param::get("/rosbag_analysis_task_node/error_codes_comments", error_codes_comments_v);
	for(int i = 0; i < error_codes_v.size(); i++)
	{
		error_codes.insert({error_codes_v[i], error_codes_comments_v[i]});
	}
	std::vector<std::string> topics;
	ros::param::get("/rosbag_analysis_task_node/topics", topics);
	ros::param::get("/rosbag_analysis_task_node/bag_file", bag_file);


	std::shared_ptr<rosbag::Bag> bag = std::make_shared<rosbag::Bag>();

	try
	{
		bag->open(bag_file, rosbag::bagmode::Read);
	}catch(rosbag::BagException){
		ROS_ERROR("Error opening file : %s", bag_file.c_str());
	}


	std::shared_ptr<rosbag::View> view = std::make_shared<rosbag::View>();
	view->addQuery(*bag, rosbag::TopicQuery(topics));

	for(rosbag::MessageInstance const msg: *view)
	{
		std_msgs::String::ConstPtr lts_error =  msg.instantiate<std_msgs::String>();
		std_msgs::Bool::ConstPtr stop_signal = msg.instantiate<std_msgs::Bool>();

		if(lts_error!= NULL)
		{
			std::cout << lts_error->data << std::endl;
		}
		if(stop_signal != NULL)
		{
			std::cout << (bool)stop_signal->data << std::endl;
		}
	}



	bag->close();

	ros::shutdown();
	

	return 0;
}