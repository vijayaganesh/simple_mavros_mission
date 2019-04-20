/*
 * File: mission_handler.cpp
 * Project: src
 * File Created: Thursday, 18th April 2019 3:41:09 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#include "simple_mavros_mission/mission_handler.hpp"

namespace simple_mission {
MissionHandler::MissionHandler(const ros::NodeHandle &nh) : nh_(nh) {
  int square_size;
  nh_.param<int>("square_path_length", square_size, 5);

  drone_ = std::make_shared<Drone>(nh_);
  mission_ = std::make_shared<SquarePathMission>(drone_, square_size);
}

MissionHandler::~MissionHandler() { ros::shutdown(); }

void MissionHandler::run() { mission_->start(); }

}  // namespace simple_mission
