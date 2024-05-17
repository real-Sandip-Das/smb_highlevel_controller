#include "smb_highlevel_controller/smb_highlevel_controller.hpp"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// STD
#include <string>
#include <cmath>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
    tfListener_(tfBuffer_)
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
  pubVis_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0 );
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

  double pillar_pos_x_ = smallest_distance*std::cos(pillarAngularPos_);
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

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_.lookupTransform("rslidar", "odom", ros::Time(0));
  } catch (tf2::TransformException &exception) {
    ROS_WARN("%s", exception.what());
    ros::Duration(1.0).sleep();
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "rslidar";
  marker.header.stamp = ros::Time();
  marker.ns = "smb_highlevel_controller";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  
  geometry_msgs::Point pos_wrt_rslidar;
  pos_wrt_rslidar.x = pillar_pos_x_;
  pos_wrt_rslidar.y = pillar_pos_y_;
  pos_wrt_rslidar.z = 0;

  // transform(pos_wrt_rslidar, marker.pose.position, transformStamped.transform);
  marker.pose.position = pos_wrt_rslidar;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  pubVis_.publish(marker);

  ROS_INFO_STREAM("Minimum Range: " << smallest_distance);
}

void SmbHighlevelController::transform(geometry_msgs::Point& pointIn, geometry_msgs::Point& pointOut, geometry_msgs::Transform transform)
{
  //translation part
  pointOut.x = pointIn.x - transform.translation.x;
  pointOut.y = pointIn.y - transform.translation.y;
  pointOut.z = pointIn.z - transform.translation.z;

  /// let, v = pointIn (3D Vector), u = (vector formed by 3D components of the quaternion in transform), s = Scalar part of the quaternion contained in transform
  double u_dot_u = std::pow(transform.rotation.x, 2) + std::pow(transform.rotation.y, 2) + std::pow(transform.rotation.y, 2);
  double u_dot_v = pointIn.x*transform.rotation.x + pointIn.y*transform.rotation.y + pointIn.z*transform.rotation.z;
  double s = transform.rotation.w;
  double s_square_minus_u_dot_u = std::pow(s, 2) - u_dot_u;

  // rotation part
  pointOut.x = 2*u_dot_v*transform.rotation.x + s_square_minus_u_dot_u*pointIn.x - 2*s*(transform.rotation.y*pointIn.z - transform.rotation.z*pointIn.y);
  pointOut.y = 2*u_dot_v*transform.rotation.y + s_square_minus_u_dot_u*pointIn.y - 2*s*(transform.rotation.z*pointIn.x - transform.rotation.x*pointIn.z);
  pointOut.z = 2*u_dot_v*transform.rotation.z + s_square_minus_u_dot_u*pointIn.z - 2*s*(transform.rotation.x*pointIn.y - transform.rotation.y*pointIn.x);
}

void SmbHighlevelController::topicCallbackPointCloud(const sensor_msgs::PointCloud2& message)
{
  int num_points = (message.row_step * message.height) / message.point_step;
  ROS_INFO_STREAM("Number of 3D points in point cloud: " << num_points);
}

} /* namespace */