/*
 * File: square_path_mission.hpp
 * Project: simple_mavros_mission
 * File Created: Thursday, 18th April 2019 9:36:32 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#ifndef SIMPLE_MAVROS_MISSION_SQUARE_PATH_MISSION_HPP_
#define SIMPLE_MAVROS_MISSION_SQUARE_PATH_MISSION_HPP_

#include <memory>
#include <vector>

#include <simple_mavros_mission/drone.hpp>
#include <simple_mavros_mission/mission.hpp>

namespace simple_mission {
/**
 * @brief This mission makes the drone travel through a square with the take off
 * point as center and land back again in the take off point.
 *
 */
class SquarePathMission : public Mission {
 public:
  /**
   * @brief Construct a new Square Path Mission object
   *
   * @param length is the length of the side of the square path
   */
  SquarePathMission(std::shared_ptr<Drone> drone_, double length);

  /**
   * @brief Destroy the Square Path Mission object
   *
   */
  virtual ~SquarePathMission();

  /**
   * @brief waypoints need for a square path is generated in this method
   *
   * @return true if the planning is successful
   * @return false if planning failed.
   */
  bool prepare_mission() override;

  /**
   * @brief Traverse the drone across the waypoints
   *
   * @return true if the mission is successful
   * @return false if the mission failed
   */
  bool run_mission() override;

  /**
   * @brief define the sequence of operation in this method
   *
   */
  void start() override;

  /**
   * @brief Compute waypoints for the mission
   *
   * @return std::vector<geometry_msgs::Pose>
   */
  std::vector<geometry_msgs::Pose> compute_waypoints();

 private:
  double length_;
  std::shared_ptr<Drone> drone;
  std::vector<geometry_msgs::Pose> way_points;
  double compute_euclidean_distance(const geometry_msgs::Pose& pose);
};
}  // namespace simple_mission

#endif  // SIMPLE_MAVROS_MISSION_SQUARE_PATH_MISSION_HPP_
