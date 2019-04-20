/*
 * File: mission_handler.hpp
 * Project: simple_mavros_mission
 * File Created: Thursday, 18th April 2019 3:47:47 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#ifndef SIMPLE_MAVROS_MISSION_MISSION_HANDLER_HPP_
#define SIMPLE_MAVROS_MISSION_MISSION_HANDLER_HPP_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <memory>
#include <string>

#include <simple_mavros_mission/drone.hpp>
#include <simple_mavros_mission/square_path_mission.hpp>

namespace simple_mission {

/**
 * @brief Main Class for the ROS Node to handle the ROS Interfacing
 *
 */
class MissionHandler {
 public:
  /**
   * @brief Construct a new Mission Handler object
   *
   */
  explicit MissionHandler(const ros::NodeHandle& nh);

  /**
   * @brief Destroy the Mission Handler object
   *
   */
  virtual ~MissionHandler();

  /**
   * @brief Run the Mission Handler
   *
   */
  void run();

 private:
  // Computation Objects
  std::shared_ptr<Drone> drone_;
  std::shared_ptr<SquarePathMission> mission_;

  // ROS Nodehandles
  ros::NodeHandle nh_;
};
}  // namespace simple_mission

#endif  // SIMPLE_MAVROS_MISSION_MISSION_HANDLER_HPP_
