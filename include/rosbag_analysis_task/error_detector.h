/**
 * \file error_detector.h
 * \brief Definition file of ErrorDetector class
 * */
#ifndef ERROR_DETECTOR_H
#define ERROR_DETECTOR_H // Include guards

// Header files
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

/**
 * \brief ErrorDetector class is used to detect the errors in the rosbag
 * */

class ErrorDetector {
public:
  /**
   * \brief Default constructor of the ErrorDetector class.
   * */
  ErrorDetector();
  /**
   * \brief Destructor of the class
   * */
  ~ErrorDetector();
  /**
   * \brief init method initializes the rosbag analysis with rosbag, rosparams
   * and creates rosbag view for detection. \return bool True if initialization
   * is success, false otherwise.
   * */
  bool init();
  /**
   * \brief detect method is used for the detection of errors in the rosbag.
   * This is the heart method of this package or node. \return bool
   * */
  bool detect();

  /**
   * \brief getErrorCodesMapping is a public getter method to get the map of
   * error codes and their comments for testing or further development. \return
   * std::map<string, string>
   * */
  std::map<std::string, std::string> getErrorCodesMapping() const;
  /**
   * \brief getBagFileNameAndLocation is a public getter method used to get the
   * bagfile name and location. \return Loaction and name of the bagfile as
   * std::string
   * */
  std::string getBagFileNameAndLocation() const;

private:
  // Map of error codes and comments.
  std::map<std::string, std::string> error_codes;

  // bagfile name and location.
  std::string bag_file;

  // Vectors that stores error codes and comments from the rosparam.
  std::vector<std::string> error_codes_v;
  std::vector<std::string> error_codes_comments_v;

  // Topic names to be used for the error detection.
  std::vector<std::string> topics;

  // Shared pointer to the rosbag
  std::shared_ptr<rosbag::Bag> bag;

  // Shared pointer to the rosbag viewer
  std::shared_ptr<rosbag::View> view;

  // Begin time of the rosbag
  ros::Time begin_time;

  // Present Stop signal state
  bool present_stop_signal;

private:
  // Private helper methods

  /**
   * \brief getMessageTime is a helper method that converts rosbag time into
   * string in the "sec.nsec" format. \return Time in string.
   * */
  std::string getMessageTime(rosbag::MessageInstance const &msg) const;
  /**
   * \brief getMessageDuration is a helper method that provides durartion of the
   * rosmessage in the rosbag in seconds (s). \return Durartion (s) in string.
   * */
  std::string getMessageDuration(rosbag::MessageInstance const &msg) const;
  /**
   * \brief writeIntoFile is a helper method that logs error dectetion message
   * into a log file name "data/report.log". \return bool
   * */
  bool writeIntoFile(std::string &logMessage);
};

#endif