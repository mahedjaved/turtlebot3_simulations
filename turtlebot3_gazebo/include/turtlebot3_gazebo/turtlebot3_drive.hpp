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
// Authors: Taehun Lim (Darby), Ryan Shim

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

class Turtlebot3Drive
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  ros::Publisher cmd_vel_pub_;

  // ROS topic subscribers
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;

  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[3];

  // ROS timer
  ros::Timer update_timer_;

  // Function prototypes
  void updateCallback(const ros::TimerEvent& event);
  void updateCmdVel(double linear, double angular);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_