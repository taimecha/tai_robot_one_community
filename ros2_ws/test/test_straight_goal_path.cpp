// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include "tai_robot_one/straight_goal_path.hpp"
#include "tai_robot_one/terminal_braking.hpp"

using nav2_costmap_2d::Costmap2D;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;

void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}

PoseStamped pose(double x, double y, double yaw)
{
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0); p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

std::vector<Point> footprint()
{
  std::vector<Point> result;
  for (const auto & xy : std::vector<std::pair<double, double>>{
      {.805, .125}, {.805, -.125}, {.405, -.275}, {-.355, -.275},
      {-.355, .275}, {.405, .275}})
  {
    Point p; p.x = xy.first; p.y = xy.second; result.push_back(p);
  }
  return result;
}

void obstacle(Costmap2D & map, double x, double y, unsigned char cost = 254)
{
  unsigned int mx, my;
  require(map.worldToMap(x, y, mx, my), "test obstacle outside map");
  map.setCost(mx, my, cost);
}

double length(const nav_msgs::msg::Path & path)
{
  double total = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

void tests()
{
  using tai_robot_one::straightGoalPath;
  const auto body = footprint();
  auto start = pose(0., 0., 0.);
  start.header.stamp.sec = 10;
  const auto goal = pose(3., 0., tai_robot_one::kPi);
  const auto free_map = []() {return Costmap2D(500, 500, .025, -5., -5., 0);};
  {
    auto map = free_map();
    auto path = straightGoalPath(start, goal, map, body, .08, {});
    require(path.has_value(), "free straight route rejected");
    require(std::abs(length(*path) - 3.) < 1e-10, "free route exceeds Euclidean length");
    for (const auto & p : path->poses) {
      require(std::abs(p.pose.position.y) < 1e-12, "straight route contains a curve");
    }
    require(std::abs(tai_robot_one::exactGoalYaw(path->poses.back().pose.orientation) -
      tai_robot_one::kPi) < 1e-10, "requested goal yaw lost");
    require(path->poses.back().header.stamp.sec == 0 &&
      path->poses.back().header.stamp.nanosec == 0, "goal freezes planning-time localization");
  }
  {
    auto map = free_map();
    for (unsigned int my = 190; my < 210; ++my) {
      for (unsigned int mx = 220; mx < 280; ++mx) {map.setCost(mx, my, 220);}
    }
    auto path = straightGoalPath(start, goal, map, body, .08, {});
    require(path && std::abs(length(*path) - 3.) < 1e-10,
      "soft inflated costs forced a curved route");
  }
  for (const auto & xy : std::vector<std::pair<double, double>>{{.9, .18}, {.75, .05}}) {
    auto map = free_map(); obstacle(map, xy.first, xy.second);
    require(!straightGoalPath(start, goal, map, body, .08, {}),
      "body/fork obstacle outside center line was ignored");
  }
  {
    auto map = free_map(); obstacle(map, 1.5, 0., 255);
    require(!straightGoalPath(start, goal, map, body, .08, {}), "unknown space was traversed");
  }
  {
    auto map = free_map(); obstacle(map, .4, .4); obstacle(map, -.4, .4);
    require(!straightGoalPath(pose(0., 0., tai_robot_one::kPi / 2.),
      pose(3., 0., 0.), map, body, .08, {}), "blocked initial sweeps were ignored");
  }
  {
    auto map = free_map(); obstacle(map, 2.20, .33); obstacle(map, 2.20, -.33);
    require(straightGoalPath(start, goal, map, body, 0., {}).has_value(),
      "exact goal sweep should be clear in early-turn fixture");
    require(!straightGoalPath(start, goal, map, body, .08, {}),
      "unsafe final turn inside XY acceptance region was ignored");
  }
  {
    auto map = free_map();
    require(straightGoalPath(start, pose(0., 0., 1.), map, body, .08, {}).has_value(),
      "stationary goal orientation rejected in free space");
    require(!straightGoalPath(pose(-4.9, 0., 0.), goal, map, body, .08, {}),
      "footprint outside map was accepted");
    int checks = 0;
    bool cancelled = false;
    try {(void)straightGoalPath(start, goal, map, body, .08, [&]() {return ++checks > 10;});}
    catch (const nav2_core::PlannerCancelled &) {cancelled = true;}
    require(cancelled, "cancelled direct search continued");
    auto bad = goal; bad.pose.orientation.w = 2.;
    bool rejected = false;
    try {(void)straightGoalPath(start, bad, map, body, .08, {});}
    catch (const nav2_core::NoValidPathCouldBeFound &) {rejected = true;}
    require(rejected, "invalid orientation accepted");
  }
}

void arrivalFrameTests()
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.header.stamp.sec = 10;
  transform.transform.rotation.w = 1.;
  require(buffer.setTransform(transform, "arrival_test"), "old map transform rejected");
  transform.header.stamp.sec = 15;
  transform.transform.translation.x = .20;
  transform.transform.translation.y = -.10;
  require(buffer.setTransform(transform, "arrival_test"), "new map transform rejected");
  const auto start = pose(0., 0., 0.);
  const auto goal = pose(3., 0., 0.);
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.stamp.sec = 10;
  path.poses = {start, pose(2.975, 0., 0.)};
  Costmap2D map(500, 500, .025, -5., -5., 0);
  tai_robot_one::appendExactGoal(path, start, goal, map, footprint(), {});
  const auto & end = path.poses.back();
  require(end.pose.position.x == goal.pose.position.x &&
    end.pose.position.y == goal.pose.position.y, "saved map goal was moved");
  require(end.header.stamp.sec == 0 && end.header.stamp.nanosec == 0,
    "grid connector goal uses old map TF");
  PoseStamped fresh, frozen;
  tf2::doTransform(end, fresh, buffer.lookupTransform("odom", "map", tf2::TimePointZero));
  auto old = end; old.header.stamp.sec = 10;
  tf2::doTransform(old, frozen, buffer.lookupTransform("odom", "map", rclcpp::Time(old.header.stamp)));
  require(std::hypot(fresh.pose.position.x - 2.8, fresh.pose.position.y - .1) < 1e-10,
    "arrival does not follow current localization");
  require(std::hypot(frozen.pose.position.x - fresh.pose.position.x,
    frozen.pose.position.y - fresh.pose.position.y) > .22,
    "stale TF fixture failed to expose inconsistent arrival positions");
  for (double sign : {-1., 1.}) {
    const double command = tai_robot_one::boundedLoadedTurnCommand(
      sign * .9, sign * .425, .22, .22, .6, .05);
    require(std::abs(command - sign * .22) < 1e-12,
      "measured angular overrun lifted command above terminal limit");
  }
}

void recordedMap(const char * filename)
{
  std::ifstream in(filename, std::ios::binary);
  unsigned int width, height;
  double resolution, x, y;
  require(static_cast<bool>(in >> width >> height >> resolution >> x >> y), "invalid map fixture header");
  in.get();
  Costmap2D map(width, height, resolution, x, y, 0);
  in.read(reinterpret_cast<char *>(map.getCharMap()), width * height);
  require(in.gcount() == width * height, "truncated map fixture");
  const auto goal = pose(3.051, 1.457, -1.5734143206728881);
  for (const auto & start : std::vector<PoseStamped>{
      pose(3.725282872382672, .04158495036427771, 1.8921413077156426),
      pose(3.4309823567642126, 1.7790492073113655, -1.5881733857455147),
      pose(3.03110525709825, .5094748887273202, -1.5668009774587621)})
  {
    auto result = tai_robot_one::straightGoalPath(start, goal, map, footprint(), .08, {});
    require(result.has_value(), "recorded static-map straight route or pivot rejected");
    const double distance = std::hypot(goal.pose.position.x - start.pose.position.x,
      goal.pose.position.y - start.pose.position.y);
    require(std::abs(length(*result) - distance) < 1e-9,
      "recorded route is longer than its straight-line lower bound");
    std::cout << "Recorded static map: straight length " << distance << " m\n";
  }
}

int main(int argc, char ** argv)
{
  try {
    tests();
    arrivalFrameTests();
    if (argc > 1) {recordedMap(argv[1]);}
    std::cout << "Straight path, full footprint, turns, inflation and cancellation checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
