/*
 * File: drone.cpp
 * Project: src
 * File Created: Thursday, 18th April 2019 3:49:28 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#include <limits>
#include <simple_mavros_mission/drone.hpp>

#define DOUBLE_MIN std::numeric_limits<double>::min()

namespace simple_mission {
Drone::Drone(const ros::NodeHandle& nh) : nh_(nh) {
  drone_state_sub_ =
      nh_.subscribe(DRONE_STATE_TOPIC, 50, &Drone::update_current_state_, this);
  drone_pose_sub_ =
      nh_.subscribe(DRONE_POSE_TOPIC, 50, &Drone::update_current_state_, this);

  setpoint_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>(DRONE_SETPOINT_TOPIC, 50);

  mode_service_client_ = nh_.serviceClient<mavros_msgs::SetMode>(MODE_SERVICE);
  arm_service_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>(ARM_SERVICE);
  param_set_service_client_ =
      nh_.serviceClient<mavros_msgs::ParamSet>(PARAM_SET_SERVICE);
  param_get_service_client_ =
      nh_.serviceClient<mavros_msgs::ParamSet>(PARAM_GET_SERVICE);
  param_pull_service_client_ =
      nh_.serviceClient<mavros_msgs::ParamPull>(PARAM_PULL_SERVICE);
}

void Drone::update_current_state_(const mavros_msgs::State state) {
  current_state_ = state;
}

void Drone::update_current_pose_(const geometry_msgs::PoseStamped drone_pose) {
  geometry_msgs::Pose current_pose = drone_pose.pose;
  current_pose_ = current_pose;
}

geometry_msgs::Pose Drone::get_current_pose() { return current_pose_; }

mavros_msgs::State Drone::get_current_state() { return current_state_; }

bool Drone::command_mode(const std::string mode) {
  mavros_msgs::SetMode mode_message;
  mode_message.request.custom_mode = mode;
  if (mode_service_client_.exists() &&
      mode_service_client_.call(mode_message)) {
    return mode_message.response.mode_sent;
  }
  return false;
}

bool Drone::pull_mavlink_param(bool is_force_pull = false) {
  mavros_msgs::ParamPull pull_message;
  pull_message.request.force_pull = is_force_pull;

  if (param_pull_service_client_.exists() &&
      param_pull_service_client_.call(pull_message)) {
    return pull_message.response.success;
  }
  return false;
}

bool Drone::set_mavlink_param(const std::string param_id, double param_value) {
  mavros_msgs::ParamSet param_message;
  mavros_msgs::ParamValue param_value_message;
  param_message.request.param_id = param_id;
  param_value_message.real = param_value;
  param_message.request.value = param_value_message;

  if (mode_service_client_.exists() &&
      mode_service_client_.call(param_message)) {
    return param_message.response.success;
  } else if (pull_mavlink_param()) {
    return set_mavlink_param(param_id, param_value);
  }
  return false;
}

bool Drone::set_xy_speed(double speed) {
  return set_mavlink_param(GROUND_SPEED_PARAM, speed);
}

bool Drone::set_takeoff_altitude(double altitude) {
  return set_mavlink_param(TAKEOFF_ALT_PARAM, altitude);
}

bool Drone::arm() {
  mavros_msgs::CommandBool arm_message;
  arm_message.request.value = true;

  if (arm_service_client_.exists() && arm_service_client_.call(arm_message)) {
    return arm_message.response.success;
  }
  return false;
}

bool Drone::return_to_land() { return command_mode(RTL); }

bool Drone::takeoff() {
  if (current_state_.armed) {
    return command_mode(TAKEOFF);
  } else {
    arm();
    return takeoff();
  }
  return false;
}

bool Drone::command_offboard() {
  ros::Rate loop_rate(10);
  int arbitrary_number = 30;
  while (arbitrary_number != 0) {
    set_pose(current_pose_);
    arbitrary_number--;
  }
  return command_mode(OFFBOARD);
}

void Drone::set_pose(const geometry_msgs::Pose& pose) {
  geometry_msgs::PoseStamped pose_message;
  pose_message.pose = pose;
  setpoint_pose_publisher_.publish(pose_message);
}

double Drone::get_mavlink_param(std::string param_id) {
  mavros_msgs::ParamGet param_msg;
  param_msg.request.param_id = param_id;
  if (param_get_service_client_.exists() &&
      param_get_service_client_.call(param_msg)) {
    return param_msg.response.value.real;
  }
  return DOUBLE_MIN;
}

}  // namespace simple_mission
