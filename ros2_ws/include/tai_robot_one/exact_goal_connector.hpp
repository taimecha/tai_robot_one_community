// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__EXACT_GOAL_CONNECTOR_HPP_
#define TAI_ROBOT_ONE__EXACT_GOAL_CONNECTOR_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tai_robot_one/rotation_sweep.hpp"

namespace tai_robot_one
{
// The native Smac2D path can end at a raster coordinate rather than at the
// requested pose. Only a short, independently checked connector is permitted.
constexpr double kExactGoalMaxConnectorM = 0.10;

inline double exactGoalYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 ||
    std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6)
  {
    throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector requires a finite planar orientation");
  }
  return std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
}

// Include polygon interiors, not only their edges: a small occupied cell
// entirely under the chassis must also reject the connector. Polygon/cell
// overlap uses separating axes; for a concave footprint it is conservative.
inline bool exactGoalCellOverlap(
  const std::vector<geometry_msgs::msg::Point> & polygon,
  double left, double bottom, double resolution)
{
  const double right = left + resolution, top = bottom + resolution;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double nx = -(b.y - a.y), ny = b.x - a.x;
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (const auto & p : polygon) {
      const double projection = nx * p.x + ny * p.y;
      low = std::min(low, projection); high = std::max(high, projection);
    }
    const double center = nx * (left + right) / 2.0 + ny * (bottom + top) / 2.0;
    const double radius = (std::abs(nx) + std::abs(ny)) * resolution / 2.0;
    if (high < center - radius - 1e-12 || low > center + radius + 1e-12) {
      return false;
    }
  }
  return true;
}

inline bool exactGoalFootprintClear(
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw, double sampling_clearance = 0.0)
{
  std::vector<geometry_msgs::msg::Point> polygon;
  double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
  double max_x = -min_x, max_y = -min_x;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto & p : footprint) {
    geometry_msgs::msg::Point q;
    q.x = x + c * p.x - s * p.y; q.y = y + s * p.x + c * p.y;
    if (!std::isfinite(q.x) || !std::isfinite(q.y)) {return false;}
    polygon.push_back(q);
    min_x = std::min(min_x, q.x); max_x = std::max(max_x, q.x);
    min_y = std::min(min_y, q.y); max_y = std::max(max_y, q.y);
  }
  // Enclose motion between neighboring translation/yaw samples as well.
  min_x -= sampling_clearance; min_y -= sampling_clearance;
  max_x += sampling_clearance; max_y += sampling_clearance;
  unsigned int first_x, first_y, last_x, last_y;
  if (polygon.size() < 3 || !map.worldToMap(min_x, min_y, first_x, first_y) ||
    !map.worldToMap(max_x, max_y, last_x, last_y))
  {
    return false;
  }
  for (unsigned int my = first_y; my <= last_y; ++my) {
    for (unsigned int mx = first_x; mx <= last_x; ++mx) {
      if (map.getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE &&
        exactGoalCellOverlap(polygon,
          map.getOriginX() + mx * map.getResolution() - sampling_clearance,
          map.getOriginY() + my * map.getResolution() - sampling_clearance,
          map.getResolution() + 2.0 * sampling_clearance))
      {
        return false;
      }
    }
  }
  return true;
}

inline void appendExactGoal(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::function<bool()> & cancel_checker)
{
  const auto cancelled = [&]() {
      if (cancel_checker && cancel_checker()) {
        throw nav2_core::PlannerCancelled("Exact-goal connector cancelled");
      }
    };
  cancelled();
  if (!std::isfinite(start.pose.position.x) || !std::isfinite(start.pose.position.y) ||
    !std::isfinite(start.pose.position.z) || path.poses.empty() || path.header.frame_id.empty() ||
    goal.header.frame_id != path.header.frame_id || start.header.frame_id != path.header.frame_id)
  {
    throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector has no path or mismatched frames");
  }
  for (const auto & pose : path.poses) {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) ||
      !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
      !std::isfinite(pose.pose.position.z))
    {
      throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector has invalid path geometry");
    }
  }
  const auto endpoint = path.poses.back().pose.position;
  const auto target = goal.pose.position;
  const double distance = std::hypot(target.x - endpoint.x, target.y - endpoint.y);
  if (!std::isfinite(distance) || !std::isfinite(target.z) ||
    distance > kExactGoalMaxConnectorM + 1e-9)
  {
    throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector exceeds its 10 cm limit or is nonfinite");
  }
  const double goal_yaw = exactGoalYaw(goal.pose.orientation);
  double incoming_yaw = exactGoalYaw(start.pose.orientation);
  for (size_t i = path.poses.size() - 1; i > 0; --i) {
    const auto & prior = path.poses[i - 1].pose.position;
    if (std::hypot(endpoint.x - prior.x, endpoint.y - prior.y) > 1e-6) {
      incoming_yaw = std::atan2(endpoint.y - prior.y, endpoint.x - prior.x);
      break;
    }
  }
  double radius = 0.0;
  for (const auto & p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector has invalid footprint");
    }
    radius = std::max(radius, std::hypot(p.x, p.y));
  }
  if (footprint.size() < 3 || radius < 1e-3 || !std::isfinite(map.getResolution()) ||
    map.getResolution() <= 0.0)
  {
    throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector has no usable padded footprint/costmap");
  }
  // Check all intermediate translations and the terminal orientation sweep.
  // The sweep is checked at every position because the controller may begin
  // its final pivot before reaching the exact endpoint.
  const double spacing = std::min(0.005, map.getResolution() / 4.0);
  const double connector_yaw = distance > 1e-6 ?
    std::atan2(target.y - endpoint.y, target.x - endpoint.x) : incoming_yaw;
  const double entry_sweep = wrapAngle(connector_yaw - incoming_yaw);
  const double final_sweep = wrapAngle(goal_yaw - connector_yaw);
  const int translations = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  const auto check_sweep = [&](double x, double y, double yaw, double sweep) {
      const int rotations = std::max(1,
        static_cast<int>(std::ceil(std::abs(sweep) * radius / spacing)));
      for (int j = 0; j <= rotations; ++j) {
        cancelled();
        if (!exactGoalFootprintClear(map, footprint, x, y,
          yaw + sweep * static_cast<double>(j) / rotations, spacing))
        {
          throw nav2_core::NoValidPathCouldBeFound("Exact-goal connector padded footprint crosses occupied/unknown/outside map");
        }
      }
    };
  for (int i = 0; i <= translations; ++i) {
    const double f = static_cast<double>(i) / translations;
    const double x = endpoint.x + f * (target.x - endpoint.x);
    const double y = endpoint.y + f * (target.y - endpoint.y);
    check_sweep(x, y, incoming_yaw, entry_sweep);
    check_sweep(x, y, connector_yaw, final_sweep);
    check_sweep(x, y, incoming_yaw, wrapAngle(goal_yaw - incoming_yaw));
  }
  auto exact = goal;
  exact.header = path.header;
  // ControllerServer transforms this pose using its own stamp on every
  // goal check. Use the latest map->odom, as the controller does while
  // following the map-frame path; a planning-time stamp freezes an old A
  // in odom after AMCL corrections. Map XY/yaw remain exactly as requested.
  exact.header.stamp = builtin_interfaces::msg::Time{};
  if (distance < 1e-9) {path.poses.back() = exact;} else {path.poses.push_back(exact);}
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__EXACT_GOAL_CONNECTOR_HPP_
