#include "smb_highlevel_controller/smb_highlevel_controller.hpp"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
  publisherCmdVel_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", publisherQueueSizeCmdVel_);
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
  if (!nodeHandle_.getParam("queue_size_cmd_vel", publisherQueueSizeCmdVel_)) return false;
  if (!nodeHandle_.getParam("k_p", k_p_)) return false;
  if (!nodeHandle_.getParam("const_lin_vel", constLinVel_)) return false;
  return true;
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
{
  float smallest_distance = message.range_max;
  int index_of_smallest_distance;
  int index = 0;
  for (float range: message.ranges) {
    if ((message.range_min <= range) && (range <= message.range_max)) {
      smallest_distance = std::min(smallest_distance, range);
      index_of_smallest_distance = index;
    }
    index++;
  }

  /// finds out the position of the pillar using the closest point in the laser scan
  double pillarAngularPos_ = message.angle_min+message.angle_increment*index_of_smallest_distance;

  // double pillar_pos_x_ = smallest_distance*std::cos(pillarAngularPos_);
  double pillar_pos_y_ = smallest_distance*std::sin(pillarAngularPos_);

  // P controller implementation
  geometry_msgs::Twist command;
  command.angular.x = 0;
  command.angular.y = 0;
  command.angular.z = k_p_*pillar_pos_y_;
  command.linear.x = constLinVel_;
  command.linear.y = 0;
  command.linear.z = 0;

  publisherCmdVel_.publish(command);

  ROS_INFO_STREAM("Minimum Range: " << smallest_distance);
}

void SmbHighlevelController::topicCallbackPointCloud(const sensor_msgs::PointCloud2& message)
{
  int num_points = (message.row_step * message.height) / message.point_step;
  ROS_INFO_STREAM("Number of 3D points in point cloud: " << num_points);
}

} /* namespace */