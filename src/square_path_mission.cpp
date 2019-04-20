/*
 * File: square_path_mission.cpp
 * Project: simple_mavros_mission
 * File Created: Thursday, 18th April 2019 6:46:59 pm
 * Author: VijayaGanesh Mohan (vmohan2@ncsu.edu)
 * -----
 * Copyright 2019 VijayaGanesh Mohan
 */

#include <ros/console.h>
#include <cmath>
#include <limits>
#include <simple_mavros_mission/square_path_mission.hpp>

#define DOUBLE_MIN std::numeric_limits<double>::min()

namespace simple_mission {

SquarePathMission::SquarePathMission(std::shared_ptr<Drone> drone_,
                                     double length)
    : drone(drone_), length_(length) {}

SquarePathMission::~SquarePathMission() {}

bool SquarePathMission::prepare_mission() {
  way_points = compute_waypoints();

  ros::Rate loop_rate(10);

  // Arm Drone
  if (!drone->arm()) {
    ROS_ERROR("Drone Not armed");
    return false;
  }

  // Takeoff Drone
  if (!drone->takeoff()) {
    ROS_ERROR("Takeoff Failed");
    return false;
    geometry_msgs::Pose way_point;
  }

  // Wait until the drone reaches the takeoff Altitude
  double flying_altitude = drone->get_mavlink_param(TAKEOFF_ALT_PARAM);
  if (flying_altitude == DOUBLE_MIN) {
    ROS_ERROR("Failed to get ROS Parmeter. Quitting");
    return false;
  }

  while (std::abs(drone->get_current_pose().position.z - flying_altitude) >=
         0.01) {
    loop_rate.sleep();
  }

  if (!drone->command_offboard()) {
    ROS_ERROR("Switching Offboard failed");
    return false;
  }
  return true;
}

bool SquarePathMission::run_mission() {
  ros::Rate loop_rate(10);
  ROS_INFO("Starting Mission");
  for (auto way_point : way_points) {
    drone->set_pose(way_point);
    ROS_INFO("Approaching Waypoint : (%f,%f)", way_point.position.x,
             way_point.position.y);
    while (compute_euclidean_distance(way_point) > 0.1) {
      if (drone->get_current_state().mode != OFFBOARD) {
        ROS_ERROR("Mode Changed in the middle of a mission");
        return false;
      }
      loop_rate.sleep();
      ROS_INFO("Distance to Destination: %f",
               compute_euclidean_distance(way_point));
    }
  }
  ROS_INFO("Mission Complete.");
  return true;
}

double SquarePathMission::compute_euclidean_distance(
    const geometry_msgs::Pose& way_point) {
  double x_diff = way_point.position.x - drone->get_current_pose().position.x;
  double y_diff = way_point.position.y - drone->get_current_pose().position.y;
  return sqrt(x_diff * x_diff + y_diff * y_diff);
}

void SquarePathMission::start() {
  while (drone->get_current_state().mode != MISSION) {
    ros::Rate loop_rate(10);
    ROS_INFO("Waiting for the user to start the mission");
    loop_rate.sleep();
  }

  // Starting the Mission Prep
  if (!prepare_mission()) {
    ROS_ERROR("Mission Prep Failed. Exiting!");
    ros::shutdown();
  }

  if (!run_mission()) {
    start();
  }

  drone->return_to_land();
}

std::vector<geometry_msgs::Pose> SquarePathMission::compute_waypoints() {
  std::vector<geometry_msgs::Pose> way_points;
  double x_min = -length_ / 2;
  double y_min = -length_ / 2;

  double x_max = length_ / 2;
  double y_max = length_ / 2;

  geometry_msgs::Pose way_point;

  // Corner 1
  way_point.position.x = x_min;
  way_point.position.y = y_min;
  way_points.push_back(way_point);

  // Corner 2
  way_point.position.x = x_max;
  way_point.position.y = y_min;
  way_points.push_back(way_point);

  // Corner 3
  way_point.position.x = x_max;
  way_point.position.y = y_max;
  way_points.push_back(way_point);

  // Corner 4
  way_point.position.x = x_min;
  way_point.position.y = y_max;
  way_points.push_back(way_point);

  return way_points;
}

}  // namespace simple_mission
