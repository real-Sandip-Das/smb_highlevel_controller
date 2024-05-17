#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace smb_highlevel_controller {

class SmbHighlevelController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SmbHighlevelController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SmbHighlevelController();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const sensor_msgs::LaserScan& message);

  /**
   * @brief ROS topic callback method for subscriber to PointCloud
   * 
   * @param message 
   */
  void topicCallbackPointCloud(const sensor_msgs::PointCloud2& message);

  /**
   * @brief Private method to do translational and rotational transformation on a Point
   * 
   * @param pointIn Input point
   * @param pointOut Reference to output point
   * @param transform The transformation to be applied
   */
  void transform(geometry_msgs::Point& pointIn, geometry_msgs::Point& pointOut, geometry_msgs::Transform transform);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic publisher for sending Twist commands to `/cmd_vel`
  ros::Publisher publisherCmdVel_;

  //! ROS topic subscriber for laser scan data.
  ros::Subscriber subscriber_;

  //! ROS topic subscriber for pointcloud data
  ros::Subscriber subscriberPointcloud_;

  //! ROS topic name to subscribe to for laser scan data.
  std::string subscriberTopic_;

  //! ROS topic name to subscribe to for pointcloud data
  std::string subscriberTopicPointcloud_;

  //! Queue size of the subscriber object subscriber_
  int subscriberQueueSize_;
  
  //! Queue size of the subscriber object subscriberPointcloud_
  int subscriberQueueSizePointcloud_;

  //! Queue size of the publisher to `/cmd_vel`
  int publisherQueueSizeCmdVel_;

  //! Proportional Controller parameter
  double k_p_;

  //! Constant linear velocity to be commanded as a part of the controller
  double constLinVel_;

  //! TF Transformation system Buffer object
  tf2_ros::Buffer tfBuffer_;

  //! TF Transform Listener object
  tf2_ros::TransformListener tfListener_;

  //! Publisher object for Visualization Marker
  ros::Publisher pubVis_;
};

} /* namespace */