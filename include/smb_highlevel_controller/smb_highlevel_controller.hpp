#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

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
};

} /* namespace */