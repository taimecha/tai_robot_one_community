// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__STRAIGHT_GOAL_PATH_HPP_
#define TAI_ROBOT_ONE__STRAIGHT_GOAL_PATH_HPP_

#include <optional>

#include "tai_robot_one/exact_goal_connector.hpp"

namespace tai_robot_one
{
// A clear straight XY route is the length lower bound. Keep its initial and
// terminal turns explicit and check the padded body AND forks, including the
// final turn throughout the controller's XY acceptance region. Inflated costs
// alone do not force a detour; lethal/unknown cells or map edges reject it.
inline std::optional<nav_msgs::msg::Path> straightGoalPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double goal_xy_tolerance, const std::function<bool()> & cancel_checker)
{
  const auto cancelled = [&]() {
      if (cancel_checker && cancel_checker()) {
        throw nav2_core::PlannerCancelled("Straight path planning cancelled");
      }
    };
  cancelled();
  if (start.header.frame_id.empty() || start.header.frame_id != goal.header.frame_id ||
    !std::isfinite(goal_xy_tolerance) || goal_xy_tolerance < 0.0 || goal_xy_tolerance > .20 ||
    !std::isfinite(map.getResolution()) || map.getResolution() <= 0.0 || footprint.size() < 3)
  {
    throw nav2_core::NoValidPathCouldBeFound("Invalid straight path frame, tolerance or footprint");
  }
  for (const auto * pose : {&start, &goal}) {
    if (!std::isfinite(pose->pose.position.x) || !std::isfinite(pose->pose.position.y) ||
      !std::isfinite(pose->pose.position.z))
    {
      throw nav2_core::NoValidPathCouldBeFound("Nonfinite straight path position");
    }
  }
  double radius = 0.0;
  for (const auto & p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw nav2_core::NoValidPathCouldBeFound("Nonfinite straight path footprint");
    }
    radius = std::max(radius, std::hypot(p.x, p.y));
  }
  if (radius < .001) {
    throw nav2_core::NoValidPathCouldBeFound("Empty straight path footprint");
  }
  const double start_yaw = exactGoalYaw(start.pose.orientation);
  const double goal_yaw = exactGoalYaw(goal.pose.orientation);
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double heading = distance > 1e-9 ? std::atan2(dy, dx) : start_yaw;
  const double spacing = std::min(.005, map.getResolution() / 4.0);
  const auto clear = [&](double x, double y, double yaw) {
      cancelled();
      return exactGoalFootprintClear(map, footprint, x, y, yaw, spacing);
    };
  const auto sweep_clear = [&](double x, double y, double from, double to) {
      return chooseRotation(wrapAngle(to - from), [&](double angle) {
          const int n = std::max(1, static_cast<int>(std::ceil(std::abs(angle) * radius / spacing)));
          for (int i = 0; i <= n; ++i) {
            if (!clear(x, y, from + angle * static_cast<double>(i) / n)) {return false;}
          }
          return true;
        }).has_value();
    };
  if (!sweep_clear(start.pose.position.x, start.pose.position.y, start_yaw, heading)) {
    return std::nullopt;
  }
  const int n = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  for (int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    const double x = start.pose.position.x + f * dx;
    const double y = start.pose.position.y + f * dy;
    if (!clear(x, y, heading) ||
      (distance * (1.0 - f) <= goal_xy_tolerance + spacing &&
      !sweep_clear(x, y, heading, goal_yaw)))
    {
      return std::nullopt;
    }
  }
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.poses.push_back(start);
  auto aligned = start;
  aligned.pose.orientation.x = aligned.pose.orientation.y = 0.0;
  aligned.pose.orientation.z = std::sin(heading / 2.0);
  aligned.pose.orientation.w = std::cos(heading / 2.0);
  if (distance > 1e-9) {
    if (std::abs(wrapAngle(heading - start_yaw)) > 1e-6) {path.poses.push_back(aligned);}
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / .025)));
    for (int i = 1; i <= samples; ++i) {
      auto pose = aligned;
      const double f = static_cast<double>(i) / samples;
      pose.pose.position.x += f * dx;
      pose.pose.position.y += f * dy;
      path.poses.push_back(pose);
    }
  }
  auto end = goal;
  end.header = path.header;
  end.header.stamp = builtin_interfaces::msg::Time{};
  path.poses.push_back(end);
  return path;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__STRAIGHT_GOAL_PATH_HPP_
