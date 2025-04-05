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

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <cmath>
#include <string>

Turtlebot3Drive::Turtlebot3Drive()
{
  ros::NodeHandle nh;

  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  scan_sub_ = nh.subscribe("scan", 10, &Turtlebot3Drive::scanCallback, this);
  odom_sub_ = nh.subscribe("odom", 10, &Turtlebot3Drive::odomCallback, this);

  /************************************************************
  ** Initialise ROS timer
  ************************************************************/
  update_timer_ = nh.createTimer(ros::Duration(0.1), &Turtlebot3Drive::updateCallback, this);

  ROS_INFO("Turtlebot3 simulation node has been initialized");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  ROS_INFO("Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::updateCmdVel(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::updateCallback(const ros::TimerEvent& event)
{
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.7;
  double check_side_dist = 0.6;

  switch (turtlebot3_state_num) {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist) {
        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        } else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        } else {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist) {
        prev_robot_pose_ = robot_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      updateCmdVel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        updateCmdVel(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        updateCmdVel(0.0, ANGULAR_VELOCITY);
      }
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_drive_node");
  Turtlebot3Drive turtlebot3_drive;
  ros::spin();

  return 0;
}