/**
 * \file error_detector.cpp
 * \brief Implementation file of the ErrorDetector class
 * */
#include "rosbag_analysis_task/error_detector.h"

#include <sstream>

ErrorDetector::ErrorDetector() : present_stop_signal{false} {
  // Get all the rosparams
  ros::param::get("/rosbag_analysis_task_node/topics", this->topics);
  ros::param::get("/rosbag_analysis_task_node/bag_file", this->bag_file);
  ros::param::get("/rosbag_analysis_task_node/error_codes",
                  this->error_codes_v);
  ros::param::get("/rosbag_analysis_task_node/error_codes_comments",
                  this->error_codes_comments_v);

  // Setup log file
  std::string pkgPath = ros::package::getPath("rosbag_analysis_task");
  std::fstream logFile;
  logFile.open((pkgPath + "/data/report.log").c_str(), std::ios::out);
  logFile << "Report of rosbag : " << this->bag_file << "\n";
  logFile << "---------------------------------\n";
  logFile.close();
}

ErrorDetector::~ErrorDetector() {}

bool ErrorDetector::init() {
  // Map the error code and comments together
  for (int i = 0; i < error_codes_v.size(); i++) {
    error_codes.insert(
        {this->error_codes_v[i], this->error_codes_comments_v[i]});
  }
  this->bag = std::make_shared<rosbag::Bag>();

  // Read the rosbag.
  try {
    bag->open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException) {
    // If not found, throw an error.
    ROS_ERROR("Error opening file : %s", bag_file.c_str());
    return false;
  }
  // Setup a rosbag view and add topics as queries.
  this->view = std::make_shared<rosbag::View>();
  this->view->addQuery(*bag, rosbag::TopicQuery(topics));
  this->begin_time = this->view->getBeginTime();

  return true;
}

bool ErrorDetector::detect() {
  for (rosbag::MessageInstance const msg : *view) {
    std_msgs::Bool::ConstPtr stop_signal = msg.instantiate<std_msgs::Bool>();
    if (stop_signal != NULL)
      this->present_stop_signal = (bool)stop_signal->data;

    std_msgs::String::ConstPtr lts_nav_error =
        msg.instantiate<std_msgs::String>();

    // Check for stop signal and non-null error message.
    if (present_stop_signal && lts_nav_error != NULL) {
      // Get the message time and duration.
      auto time_string = getMessageTime(msg);
      auto duration_string = getMessageDuration(msg);
      auto it = error_codes.find(lts_nav_error->data.c_str());
      std::string reason_for_error;
      if (it != error_codes.end())
        reason_for_error = it->second;
      reason_for_error.append(" @ TimeStamp: ");
      reason_for_error.append(time_string);
      reason_for_error.append(" , ROS bag duration: ");
      reason_for_error.append(duration_string);

      // Write it into a log file.
      writeIntoFile(reason_for_error);
      // std::cout << reason_for_error << std::endl;
    }
  }
  // Close the bag
  this->bag->close();

  return true;
}

std::string
ErrorDetector::getMessageTime(rosbag::MessageInstance const &msg) const {
  auto time = msg.getTime();
  std::stringstream ss;
  ss << time.sec << "." << time.nsec;
  std::string time_string = ss.str();
  return time_string;
}

std::string
ErrorDetector::getMessageDuration(rosbag::MessageInstance const &msg) const {
  auto duration = (msg.getTime() - this->begin_time).toSec();

  return std::to_string(duration).append("(s)");
}

bool ErrorDetector::writeIntoFile(std::string &logMessage) {
  std::string pkgPath = ros::package::getPath("rosbag_analysis_task");
  std::fstream logFile;
  logFile.open((pkgPath + "/data/report.log").c_str(),
               std::ios::app | std::ios::out);
  if (!logFile.is_open()) {
    ROS_ERROR("Could not open report.log");
    return false;
  }
  logFile << logMessage << "\n";
  logFile.close();
  return true;
}

std::map<std::string, std::string> ErrorDetector::getErrorCodesMapping() const {
  return this->error_codes;
}

std::string ErrorDetector::getBagFileNameAndLocation() const {
  return this->bag_file;
}