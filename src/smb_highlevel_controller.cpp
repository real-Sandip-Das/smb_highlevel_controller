#include "smb_highlevel_controller/smb_highlevel_controller.hpp"
#include <ros/ros.h>

// STD
#include <string>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &SmbHighlevelController::topicCallback, this);
  ROS_INFO("Successfully launched node.");
}

SmbHighlevelController::~SmbHighlevelController()
{
}

bool SmbHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
{
  //TODO: print the smallest distance measurement from the vector ranges in the message of the laser scanner to the terminal
}

} /* namespace */