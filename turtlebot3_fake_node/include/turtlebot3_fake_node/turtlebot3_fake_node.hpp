// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Yoonseok Pyo, Ryan Shim

#ifndef TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_
#define TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <turtlebot3_msgs/SensorState.h>

#define LEFT 0
#define RIGHT 1

class Turtlebot3Fake
{
public:
  Turtlebot3Fake();
  ~Turtlebot3Fake();

private:
  // ROS time
  ros::Time last_cmd_vel_time_;
  ros::Time prev_update_time_;

  // ROS timer
  ros::Timer update_timer_; // Add this declaration

  // ROS publishers
  ros::Publisher odom_pub_;
  ros::Publisher joint_states_pub_;
  ros::Publisher tf_pub_;

  // ROS subscribers
  ros::Subscriber cmd_vel_sub_;

  nav_msgs::Odometry odom_;
  sensor_msgs::JointState joint_states_;

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;
  double last_position_[2];
  double last_velocity_[2];
  float odom_pose_[3];
  float odom_vel_[3];

  double wheel_separation_;
  double wheel_radius_;

  // Function prototypes
  void initParameters();
  void initVariables();
  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);
  void updateCallback(const ros::TimerEvent& event);
  bool updateOdometry(const ros::Duration& diff_time);
  void updateJointState();
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

#endif  // TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_