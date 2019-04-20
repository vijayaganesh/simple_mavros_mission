/*
 * File: mission.hpp
 * Project: simple_mavros_mission
 * File Created: Thursday, 18th April 2019 4:05:37 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#ifndef SIMPLE_MAVROS_MISSION_MISSION_HPP
#define SIMPLE_MAVROS_MISSION_MISSION_HPP

namespace simple_mission {
/**
 * @brief Simple Abstract class to define a mission
 *
 */
class Mission {
 public:
  /**
   * @brief Construct a new Mission object
   *
   */
  Mission();

  /**
   * @brief Destroy the Mission object
   *
   */
  virtual ~Mission();

  /**
   * @brief The required pre processing to carryout a mission is done in this
   * function Eg: Computing the setpoints required for a mission.
   *
   * @return true if the preparation is successful
   * @return false if the preparation failed
   */
  virtual bool prepare_mission();

  /**
   * @brief This function is called to run the mission
   *
   * @return true if the mission is successful
   * @return false if the mission failed
   */
  virtual bool run_mission() = 0;

  /**
   * @brief The required post processing to be done after the mission is
   * completed is done in this function
   *
   * @return true if the post processing is successful.
   * @return false if the post processing failed.
   */
  virtual bool end_mission();

  /**
   * @brief This method starts the mission and runs the prepare, run and end
   * functions (if exists) sequentially.
   *
   */
  virtual void start() = 0;
};
}  // namespace simple_mission

#endif