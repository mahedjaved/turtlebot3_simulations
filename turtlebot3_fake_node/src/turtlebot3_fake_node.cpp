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

#include "turtlebot3_fake_node/turtlebot3_fake_node.hpp"

#include <cmath>
#include <string>

Turtlebot3Fake::Turtlebot3Fake()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  /************************************************************
  ** Initialise ROS parameters
  ************************************************************/
  private_nh.param<std::string>("joint_states_frame", joint_states_.header.frame_id, "base_footprint");
  private_nh.param<std::string>("odom_frame", odom_.header.frame_id, "odom");
  private_nh.param<std::string>("base_frame", odom_.child_frame_id, "base_footprint");
  private_nh.param<double>("wheels.separation", wheel_separation_, 0.0);
  private_nh.param<double>("wheels.radius", wheel_radius_, 0.0);

  /************************************************************
  ** Initialise variables
  ************************************************************/
  initVariables();

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  tf_pub_ = nh.advertise<tf2_msgs::TFMessage>("tf", 10);

  cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &Turtlebot3Fake::commandVelocityCallback, this);

  /************************************************************
  ** Initialise ROS timer
  ************************************************************/
  update_timer_ = nh.createTimer(ros::Duration(0.01), &Turtlebot3Fake::updateCallback, this);

  ROS_INFO("Turtlebot3 fake node has been initialised");
}

Turtlebot3Fake::~Turtlebot3Fake()
{
  ROS_INFO("Turtlebot3 fake node has been terminated");
}

/********************************************************************************
** Init functions
********************************************************************************/
void Turtlebot3Fake::initVariables()
{
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_ = 0.0;
  goal_angular_velocity_ = 0.0;
  cmd_vel_timeout_ = 1.0;
  last_position_[LEFT] = 0.0;
  last_position_[RIGHT] = 0.0;
  last_velocity_[LEFT] = 0.0;
  last_velocity_[RIGHT] = 0.0;

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back("wheel_left_joint");
  joint_states_.name.push_back("wheel_right_joint");
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  prev_update_time_ = ros::Time::now();
  last_cmd_vel_time_ = ros::Time::now();
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Fake::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT] = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_separation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_separation_ / 2);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Fake::updateCallback(const ros::TimerEvent& event)
{
  ros::Time time_now = ros::Time::now();
  ros::Duration duration = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // Zero-ing after timeout (stop the robot if no cmd_vel)
  if ((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_) {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // Odom
  updateOdometry(duration);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // Joint states
  updateJointState();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // TF
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf2_msgs::TFMessage odom_tf_msg;
  odom_tf_msg.transforms.push_back(odom_tf);
  tf_pub_.publish(odom_tf_msg);
}

bool Turtlebot3Fake::updateOdometry(const ros::Duration& diff_time)
{
  double wheel_l, wheel_r;  // Rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];
  double step_time = diff_time.toSec();  // [sec]

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / wheel_radius_;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius_;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * step_time;
  wheel_r = w[RIGHT] * step_time;

  if (std::isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (std::isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_separation_;

  // Compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = delta_s / step_time;     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / step_time;  // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose_[2]);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  // Update the twist of the odometry
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}

void Turtlebot3Fake::updateJointState()
{
  joint_states_.position[LEFT] = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT] = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

void Turtlebot3Fake::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_fake_node");
  Turtlebot3Fake turtlebot3_fake;
  ros::spin();

  return 0;
}