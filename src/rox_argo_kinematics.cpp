/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <neo_msgs/KinematicsState.h>
#include <angles/angles.h>
#include <std_msgs/Header.h>
#include <mutex>
#include <string>

#include "../include/rox_argo_kinematics/OmniKinematics.h"
#include "../include/rox_argo_kinematics/VelocitySolver.h"

class ArgoKinematicsNode {
public:
  ArgoKinematicsNode() {
    // Load parameters
    nh.param("control_rate", m_control_rate, 50.0);
    nh.param("broadcast_tf", m_broadcast_tf, true);
    nh.param("num_wheels", m_num_wheels, 4);
    nh.param("wheel_radius", m_wheel_radius, 0.0);
    nh.param("cmd_timeout", m_cmd_timeout, 0.1);
    nh.param("wheel_lever_arm", m_wheel_lever_arm, 0.0);
    nh.param("zero_vel_threshold", m_zero_vel_threshold, 0.001);
    nh.param("small_vel_threshold", m_small_vel_threshold, 0.0);
    nh.param("steer_hysteresis", m_steer_hysteresis, 30.0);
    nh.param("steer_hysteresis_dynamic", m_steer_hysteresis_dynamic, 5.0);

    if (m_num_wheels < 2) {
      throw std::logic_error("invalid num_wheels param");
    }
    m_wheels.resize(m_num_wheels);
    for (int i = 0; i < m_num_wheels; ++i) {
      m_wheels[i].lever_arm = m_wheel_lever_arm;
      nh.param<std::string>("wheel" + std::to_string(i) + "/drive_joint_name", m_wheels[i].drive_joint_name, "random");
      nh.param<std::string>("wheel" + std::to_string(i) + "/steer_joint_name", m_wheels[i].steer_joint_name, "random");
      nh.param("wheel" + std::to_string(i) + "/center_pos_x", m_wheels[i].center_pos_x, 1.0);
      nh.param("wheel" + std::to_string(i) + "/center_pos_y", m_wheels[i].center_pos_y, 1.0);
      m_wheels[i].home_angle = M_PI * m_wheels[i].home_angle / 180.;
      m_wheels[i].set_wheel_angle(0);
    }
    m_kinematics = std::make_shared<OmniKinematics>(m_num_wheels);
    m_velocity_solver = std::make_shared<VelocitySolver>(m_num_wheels);
    m_kinematics->zero_vel_threshold = m_zero_vel_threshold;
    m_kinematics->small_vel_threshold = m_small_vel_threshold;
    m_kinematics->steer_hysteresis = M_PI * m_steer_hysteresis / 180;
    m_kinematics->steer_hysteresis_dynamic = M_PI * m_steer_hysteresis_dynamic / 180;
    m_kinematics->initialize(m_wheels);
    m_pub_odometry = nh.advertise<nav_msgs::Odometry>("odom", 1);
    m_pub_joint_trajectory = nh.advertise<trajectory_msgs::JointTrajectory>("drive/joint_trajectory", 1);
    m_pub_kinematics_state = nh.advertise<neo_msgs::KinematicsState>("kinematics_state", 1);
    m_sub_cmd_vel = nh.subscribe("cmd_vel", 1, &ArgoKinematicsNode::cmd_vel_callback, this);
    m_sub_joint_state = nh.subscribe("drive/joint_states", 1, &ArgoKinematicsNode::joint_state_callback, this);
  }

  void control_step() {
    std::lock_guard<std::mutex> lock(m_node_mutex);
    ros::Time now = ros::Time::now();
    if ((now - m_last_cmd_time).toSec() > m_cmd_timeout) {
      if (!is_cmd_timeout && m_last_cmd_time.toSec() != 0 &&
        (m_last_cmd_vel.linear.x != 0 || m_last_cmd_vel.linear.y != 0 ||
        m_last_cmd_vel.angular.z != 0)) {
        ROS_WARN("cmd_vel input timeout! Stopping now.");
      }
      is_cmd_timeout = true;
    } else {
      is_cmd_timeout = false;
    }
    auto cmd_wheels = m_kinematics->compute(
      m_wheels, m_last_cmd_vel.linear.x,
      m_last_cmd_vel.linear.y, m_last_cmd_vel.angular.z);
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = now;
    trajectory_msgs::JointTrajectoryPoint point;
    for (const auto & wheel : cmd_wheels) {
      joint_trajectory.joint_names.push_back(wheel.drive_joint_name);
      joint_trajectory.joint_names.push_back(wheel.steer_joint_name);
      {
        const double drive_rot_vel = wheel.wheel_vel / m_wheel_radius;
        point.positions.push_back(0);
        point.velocities.push_back(drive_rot_vel);
      }
      {
        point.positions.push_back(wheel.wheel_angle);
        point.velocities.push_back(0);
      }
    }
    joint_trajectory.points.push_back(point);
    m_pub_joint_trajectory.publish(joint_trajectory);
  }

  double get_control_rate() const { return m_control_rate; }

private:
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist) {
    std::lock_guard<std::mutex> lock(m_node_mutex);
    m_last_cmd_time = ros::Time::now();
    m_last_cmd_vel.linear.x = twist->linear.x;
    m_last_cmd_vel.linear.y = twist->linear.y;
    m_last_cmd_vel.angular.z = twist->angular.z;
  }

  void joint_state_callback(const sensor_msgs::JointState::ConstPtr& joint_state) {
    std::lock_guard<std::mutex> lock(m_node_mutex);
    geometry_msgs::Quaternion quat_msg1;
    const size_t num_joints = joint_state->name.size();
    if (joint_state->position.size() < num_joints) {
      ROS_ERROR_ONCE("joint_state->position.size() < num_joints");
      return;
    }
    if (joint_state->velocity.size() < num_joints) {
      ROS_ERROR_ONCE("joint_state->velocity.size() < num_joints");
      return;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      for (auto & wheel : m_wheels) {
        if (joint_state->name[i] == wheel.drive_joint_name) {
          wheel.wheel_vel = -1 * joint_state->velocity[i] * m_wheel_radius;
        }
        if (joint_state->name[i] == wheel.steer_joint_name) {
          wheel.set_wheel_angle(joint_state->position[i] + M_PI);
        }
      }
    }
    m_velocity_solver->solve(m_wheels);
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "odom";
    odometry.header.stamp = joint_state->header.stamp;
    odometry.child_frame_id = "base_link";
    if (!m_curr_odom_time.isZero()) {
      const double dt = (joint_state->header.stamp - m_curr_odom_time).toSec();
      if (dt > 0 && dt < 1) {
        const double vel_x_mid = 0.5 * (m_velocity_solver->move_vel_x + m_curr_odom_twist.linear.x);
        const double vel_y_mid = 0.5 * (m_velocity_solver->move_vel_y + m_curr_odom_twist.linear.y);
        const double yawrate_mid = 0.5 * (m_velocity_solver->move_yawrate + m_curr_odom_twist.angular.z);
        const double yaw_mid = m_curr_odom_yaw + 0.5 * yawrate_mid * dt;
        m_curr_odom_x += vel_x_mid * dt * cos(yaw_mid) + vel_y_mid * dt * -sin(yaw_mid);
        m_curr_odom_y += vel_x_mid * dt * sin(yaw_mid) + vel_y_mid * dt * cos(yaw_mid);
        m_curr_odom_yaw += yawrate_mid * dt;
      } else {
        ROS_WARN("invalid joint state delta time");
      }
    }
    m_curr_odom_time = joint_state->header.stamp;
    odometry.pose.pose.position.x = m_curr_odom_x;
    odometry.pose.pose.position.y = m_curr_odom_y;
    odometry.pose.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, m_curr_odom_yaw);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    m_curr_odom_twist.linear.x = m_velocity_solver->move_vel_x;
    m_curr_odom_twist.linear.y = m_velocity_solver->move_vel_y;
    m_curr_odom_twist.linear.z = 0;
    m_curr_odom_twist.angular.x = 0;
    m_curr_odom_twist.angular.y = 0;
    m_curr_odom_twist.angular.z = m_velocity_solver->move_yawrate;
    odometry.twist.twist = m_curr_odom_twist;
    for (int i = 0; i < 36; ++i) {
      odometry.pose.covariance[i] = 0.1;
      odometry.twist.covariance[i] = 0.1;
    }
    m_pub_odometry.publish(odometry);
    if (m_broadcast_tf) {
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.stamp = joint_state->header.stamp;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";
      odom_tf.transform.translation.x = m_curr_odom_x;
      odom_tf.transform.translation.y = m_curr_odom_y;
      odom_tf.transform.translation.z = 0;
      odom_tf.transform.rotation.x = q.x();
      odom_tf.transform.rotation.y = q.y();
      odom_tf.transform.rotation.z = q.z();
      odom_tf.transform.rotation.w = q.w();
      m_tf_odom_broadcaster.sendTransform(odom_tf);
    }
    m_kinematics_state.is_moving = false;
    m_kinematics_state.is_vel_cmd = false;
    if (m_last_cmd_vel.linear.x != 0 ||
      m_last_cmd_vel.linear.y != 0 ||
      m_last_cmd_vel.angular.z != 0) {
      m_kinematics_state.is_vel_cmd = true;
    }
    if(m_curr_odom_twist.linear.x != 0 ||
      m_curr_odom_twist.linear.y != 0 ||
      m_curr_odom_twist.angular.z != 0) 
    {
      m_kinematics_state.is_moving = true;
    }
    m_pub_kinematics_state.publish(m_kinematics_state);
  }

  std::mutex m_node_mutex;
  ros::Publisher m_pub_odometry;
  ros::Publisher m_pub_joint_trajectory;
  ros::Publisher m_pub_kinematics_state;
  ros::Subscriber m_sub_cmd_vel;
  ros::Subscriber m_sub_joint_state;
  tf::TransformBroadcaster m_tf_odom_broadcaster;
  bool m_broadcast_tf = false;
  int m_num_wheels = 0;
  int m_steer_reset_button = -1;
  double m_wheel_radius = 0;
  double m_wheel_lever_arm = 0;
  double m_cmd_timeout = 0;
  double m_control_rate = 0;
  double m_zero_vel_threshold = 0.005;
  double m_small_vel_threshold = 0.03;
  double m_steer_hysteresis = 30.0;
  double m_steer_hysteresis_dynamic = 5.0;
  std::vector<OmniWheel> m_wheels;
  std::shared_ptr<OmniKinematics> m_kinematics;
  std::shared_ptr<VelocitySolver> m_velocity_solver;
  ros::Time m_last_cmd_time;
  geometry_msgs::Twist m_last_cmd_vel;
  bool is_cmd_timeout = false;
  ros::Time m_curr_odom_time;
  double m_curr_odom_x = std::numeric_limits<double>::min();
  double m_curr_odom_y = std::numeric_limits<double>::min();
  double m_curr_odom_yaw = std::numeric_limits<double>::min();
  geometry_msgs::Twist m_curr_odom_twist;
  neo_msgs::KinematicsState m_kinematics_state;
  ros::NodeHandle nh;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rox_argo_kinematics");
  ros::NodeHandle nh;
  ArgoKinematicsNode node;
  ros::Rate loop_rate(node.get_control_rate());
  ROS_INFO("Starting the ROX kinematics node");
  while (ros::ok()) {
    ros::spinOnce();
    node.control_step();
    loop_rate.sleep();
  }
  return 0;
}
