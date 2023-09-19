/*
 * Author: Victor W H Wu
 * Date: 12 January 2023.
 * Description:
 * 
 * File Path: chsweld/src/bunker_pro/src/bunker_pro_base_sim_node.cpp
 * 
 * Copyright (c) 2023 Victor W H Wu
 */

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "bunker_pro/bunker_pro_messenger.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "bunker_odom");
  ros::NodeHandle node(""), private_node("~");

  // instantiate a robot object
  // BunkerRobot robot;
  std::unique_ptr<BunkerRobot> bunker;

  // instantiate a messenger object with &robot and &node
  BunkerROSMessenger messenger(&bunker, &node);

  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("bunker_pro_base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, true);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<bool>("publish_tf", messenger.pub_tf_,  true);

  // setup ROS subscription and publication topics and publishers and subscribers.
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  double linear, angular;
  ros::Rate rate_50hz(50);  // 50Hz
  while (ros::ok()) {
    messenger.GetCurrentMotionCmdForSim(linear, angular);
    messenger.PublishSimStateToROS(linear, angular);
    ros::spinOnce();
    rate_50hz.sleep();
  }

  return 0;
}