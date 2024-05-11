#include "smb_highlevel_controller/smb_highlevel_controller.hpp"

// ROS
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
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, subscriberQueueSize_,
                                      &SmbHighlevelController::topicCallback, this);
  subscriberPointcloud_ = nodeHandle_.subscribe(subscriberTopicPointcloud_, subscriberQueueSizePointcloud_,
                                                &SmbHighlevelController::topicCallbackPointCloud, this);
  ROS_INFO("Successfully launched node.");
}

SmbHighlevelController::~SmbHighlevelController()
{
}

bool SmbHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("topic_name", subscriberTopic_)) return false;
  if (!nodeHandle_.getParam("queue_size", subscriberQueueSize_)) return false;
  if (!nodeHandle_.getParam("topic_name_pointcloud", subscriberTopicPointcloud_)) return false;
  if (!nodeHandle_.getParam("queue_size_pointcloud", subscriberQueueSizePointcloud_)) return false;
  return true;
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
{
  float smallest_distance = message.range_max;
  for (float range: message.ranges) {
    if ((message.range_min <= range) && (range <= message.range_max)) {
      smallest_distance = std::min(smallest_distance, range);
    }
  }

  ROS_INFO_STREAM("Minimum Range: " << smallest_distance);
}

void SmbHighlevelController::topicCallbackPointCloud(const sensor_msgs::PointCloud2& message)
{
  int num_points = (message.row_step * message.height) / message.point_step;
  ROS_INFO_STREAM("Number of 3D points in point cloud: " << num_points);
}

} /* namespace */