#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

};

} /* namespace */