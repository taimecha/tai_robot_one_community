// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__PATH_TRACKING_HPP_
#define TAI_ROBOT_ONE__PATH_TRACKING_HPP_
#include <cmath>
#include <limits>
#include <vector>
#include "tai_robot_one/rotation_sweep.hpp"
namespace tai_robot_one
{
struct TrackingPose {double x, y, yaw;};
struct TrackingTarget
{
  double initial_angle{0.0};
  double lookahead_limit{std::numeric_limits<double>::infinity()};
  double planned_turn{0.0};
  bool at_turn{false};
};

// A bounded rear goal is a normal route. Heading describes the chassis,
// while every translated segment moves opposite that heading.
inline bool shortReversePath(const std::vector<TrackingPose> & path)
{
  if (path.size() < 2) {return false;}
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1]; const auto & b = path[i];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.yaw) ||
      !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.yaw)) {return false;}
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double d = std::hypot(dx, dy);
    if (d < 1e-6) {continue;}
    if (std::abs(wrapAngle(std::atan2(dy, dx) - a.yaw - kPi)) > .12) {return false;}
    length += d;
  }
  return length > .01 && length <= 1.5 &&
         std::hypot(path.back().x - path.front().x, path.back().y - path.front().y) <= 1.0 + 1e-6;
}

inline bool canApproachGoalWithoutStationaryTurn(
  double x, double y, double remaining_length, double xy_tolerance)
{
  // Close to arrival, a short forward tracking arc can enter XY tolerance
  // before final alignment. Avoid a needless turn toward the exact point.
  return remaining_length <= 0.15 && x > 0.0 &&
         std::hypot(x, y) <= 0.15 && std::abs(y) < 0.9 * xy_tolerance &&
         std::abs(std::atan2(y, x)) < 0.80;
}

inline bool terminalReverseCorrectionAllowed(
  double x, double y, double remaining_length, double xy_tolerance,
  double yaw_error, bool arrived_before, bool allow_new_near_goal = false)
{
  // Normal profiles require previous arrival. The A profile can explicitly
  // opt into a new, shorter rear correction. Straight retreat preserves the
  // accepted lateral error and heading; the caller checks the full corridor.
  const double limit = arrived_before ? 0.15 : 0.12;
  return (arrived_before || allow_new_near_goal) && remaining_length <= limit && x < -xy_tolerance &&
         std::hypot(x, y) <= limit && std::abs(y) < 0.9 * xy_tolerance &&
         std::abs(yaw_error) <= 0.35;
}

inline TrackingTarget trackingTarget(const std::vector<TrackingPose> & path)
{
  TrackingTarget target;
  size_t travel_start = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    if (std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) < 0.005 &&
      std::abs(wrapAngle(path[i].yaw - path[i - 1].yaw)) > 0.02)
    {
      size_t end = i;
      while (end + 1 < path.size() &&
        std::hypot(path[end + 1].x - path[i].x, path[end + 1].y - path[i].y) < 0.005)
      {
        ++end;
      }
      const double distance = std::hypot(path[i].x, path[i].y);
      // A skid turn can drift a few cm. Once aligned, release the cusp within
      // the 5 cm positional tolerance instead of chasing it and turning again.
      if (distance <= 0.05 && std::abs(wrapAngle(path[end].yaw)) <= 0.04) {
        travel_start = end;
        i = end;
        continue;
      }
      target.lookahead_limit = distance;
      target.at_turn = distance <= 0.015;
      target.planned_turn = wrapAngle(path[end].yaw);
      break;
    }
  }
  // Select the travel heading AFTER releasing completed turns. The old cusp
  // can be behind the robot after skid drift; its bearing must never become
  // a new 180-degree turn target.
  for (size_t i = travel_start; i < path.size(); ++i) {
    if (std::hypot(path[i].x, path[i].y) >= 0.05) {
      target.initial_angle = std::atan2(path[i].y, path[i].x);
      break;
    }
  }
  // Near the route, follow the immediate ~0.25 m travel chord so initial
  // heading alignment agrees with Pure Pursuit lookahead instead of fighting
  // single 5 cm Smac2D grid steps. Stop early at an unfinished rotation cusp.
  if (path.size() > travel_start + 1 &&
    (travel_start > 0 || std::hypot(path.front().x, path.front().y) <= 0.25))
  {
    const auto & origin = path[travel_start];
    for (size_t i = travel_start + 1; i < path.size(); ++i) {
      const double step_d = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      if (step_d < 0.005 && std::abs(wrapAngle(path[i].yaw - path[i - 1].yaw)) > 0.02) {
        break;
      }
      const double chord_dx = path[i].x - origin.x;
      const double chord_dy = path[i].y - origin.y;
      const double chord_d = std::hypot(chord_dx, chord_dy);
      if (chord_d >= 0.005) {
        target.initial_angle = std::atan2(chord_dy, chord_dx);
      }
      if (chord_d >= 0.25) {
        break;
      }
    }
  }
  return target;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__PATH_TRACKING_HPP_
