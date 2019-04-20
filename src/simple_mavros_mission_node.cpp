/*
 * File: simple_mavros_mission_node.cpp
 * Project: simple_mavros_mission
 * File Created: Friday, 19th April 2019 6:26:44 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#include <ros/ros.h>
#include "simple_mavros_mission/mission_handler.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "Simple_Mavros_Mission");
  ros::NodeHandle node_handle("~");
  simple_mission::MissionHandler mission_handler(node_handle);
  ros::spin();
  return 0;
}
