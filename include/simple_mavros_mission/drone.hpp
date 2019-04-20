/*
 * File: drone.hpp
 * Project: simple_mavros_mission
 * File Created: Thursday, 18th April 2019 3:48:08 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#ifndef SIMPLE_MAVROS_MISSION_DRONE_HPP_
#define SIMPLE_MAVROS_MISSION_DRONE_HPP_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <string>

#define TAKEOFF_ALT_PARAM "MIS_TAKEOFF_ALT"
#define GROUND_SPEED_PARAM "MPC_XY_VEL_MAX"

#define DRONE_STATE_TOPIC "/mavros/state"
#define DRONE_POSE_TOPIC "/mavros/local_position/pose"
#define DRONE_SETPOINT_TOPIC "/mavros/setpoint_position/local"

#define MODE_SERVICE "/mavros/set_mode"
#define ARM_SERVICE "/mavros/cmd/arming"
#define PARAM_SET_SERVICE "/mavros/param/set"
#define PARAM_GET_SERVICE "/mavros/param/get"
#define PARAM_PULL_SERVICE "/mavros/param/pull"

// AUTO Flight Modes
#define LOITER "AUTO.LOITER"
#define LAND "AUTO.LAND"
#define TAKEOFF "AUTO.TAKEOFF"
#define MISSION "AUTO.MISSION"
#define RTL "AUTO.RTL"
#define READY "AUTO.READY"
#define RTGS "AUTO.RTGS"
#define OFFBOARD "OFFBOARD"

// ASSISTED Flight Modes
#define ALTCTL "ALTCTL"
#define POSCTL "POSCTL"

// MANUAL Flight Modes
#define ACRO "ACRO"
#define STABILIZED "STABILIZED"
#define RATTITUDE "RATTITUDE"
#define MANUAL "MANUAL"

namespace simple_mission {
/**
 * @brief Class to maintain the drone state
 *
 */
class Drone {
 public:
  /**
   * @brief Default Constructor of Drone Class.
   *
   */
  explicit Drone(const ros::NodeHandle& nh);

  /**
   * @brief Destroy the Drone object.
   *
   */
  virtual ~Drone();

  /**
   * @brief Force the drone a particular mode.
   *
   * @param mode One of
   *         MANUAL
   *         ACRO
   *         ALTCTL
   *         POSCTL
   *         OFFBOARD
   *         STABILIZED
   *         RATTITUDE
   *         MISSION
   *         LOITER
   *         RTL
   *         LAND
   *         RTGS
   *         READY
   *         TAKEOFF
   *
   *
   * @return true if the drone is successfully forced to offboard control.
   * @return false if offboard control failed.
   */
  bool command_mode(const std::string mode);

  /**
   * @brief Set the mavlink param value
   *
   * @param param_id Available Param IDS can be found in
   *              https://dev.px4.io/en/advanced/parameter_reference.html
   * @param param_value Value to be set for the parameter
   * @return true if the parameter is set successfully
   * @return false if parameter setting failed.
   */
  bool set_mavlink_param(const std::string param_id, double param_value);

  /**
   * @brief Sometimes, MavROS cannot set parameter directly to MAVLINK and
   *        would require first to pull the parameters from the cache and
   *        then set the the parameters. This function is to fetch params.
   *
   * @param is_force_pull
   * @return true if pulling is successful.
   * @return false if pulling failed.
   */
  bool pull_mavlink_param(bool is_force_pull);

  /**
   * @brief Set the ground speed of the drone
   *
   * @param speed is the ground speed at which the drone must fly
   * @return true if the speed is succesfully set.
   * @return false if set speed failed
   */
  bool set_xy_speed(double speed);

  /**
   * @brief Set the takeoff altitude of the drone
   *
   * @param speed is the take off altitude at which the drone must fly
   * @return true if the altitude is succesfully set.
   * @return false if set altitude failed
   */
  bool set_takeoff_altitude(double altitude);

  /**
   * @brief Call this function to arm the drone
   *
   * @return true if arming is successful
   * @return false if arming failed
   */
  bool arm();

  /**
   * @brief Command the drone to offboard mode
   *
   * @return true if switching is successful.
   * @return false if switching failed.
   */
  bool command_offboard();

  /**
   * @brief Get the mavlink parameter from drone
   *
   * @param param_id Available Param IDS can be found in
   *              https://dev.px4.io/en/advanced/parameter_reference.html
   * @return double
   */
  double get_mavlink_param(std::string param_id);

  /**
   * @brief Set the desired pose of the drone.
   *        Note: This pose will be published as a ROS Message
   *
   * @param pose is the desired pose of type Geometry_msgs/PoseStamped.msg
   */
  void set_pose(const geometry_msgs::Pose& pose);

  /**
   * @brief Call this function to initiate landing.
   *
   * @return true if landing is successful
   * @return false if landing failed
   */
  bool return_to_land();

  /**
   * @brief Call this function to initiate takeoff.
   *
   * @return true if takeoff is successful
   * @return false if takeoff failed
   */
  bool takeoff();

  /**
   * @brief Get the current pose of the drone
   *
   * @return geometry_msgs::Pose
   */
  geometry_msgs::Pose get_current_pose();

  /**
   * @brief Get the current state of the drone
   *
   * @return mavros_msgs::State
   */
  mavros_msgs::State get_current_state();

 private:
  geometry_msgs::Pose current_pose_;
  mavros_msgs::State current_state_;

  // ROS Nodehandles
  ros::NodeHandle nh_;

  // ROS Publishers
  ros::Publisher setpoint_pose_publisher_;

  // ROS Subscribers
  ros::Subscriber drone_state_sub_;
  ros::Subscriber drone_pose_sub_;

  // ROS Service Clients
  ros::ServiceClient mode_service_client_;
  ros::ServiceClient arm_service_client_;
  ros::ServiceClient param_set_service_client_;
  ros::ServiceClient param_get_service_client_;
  ros::ServiceClient param_pull_service_client_;

  /**
   * @brief Update the current pose of the drone
   *
   * @param pose is the current pose obtained from a ROS Topic
   */
  void update_current_pose_(const geometry_msgs::PoseStamped drone_pose);

  /**
   * @brief Update the current state of the drone
   *
   * @param state is the current state obtained from a ROS Topic
   */
  void update_current_state_(const mavros_msgs::State state);
};
}  // namespace simple_mission

#endif  // SIMPLE_MAVROS_MISSION_DRONE_HPP_
