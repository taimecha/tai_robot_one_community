// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tai_robot_one/rotation_sweep.hpp"
#include "tai_robot_one/path_tracking.hpp"

namespace tai_robot_one
{
// RPP's translation regulation is retained. Heading alignment additionally
// checks the complete swept footprint, not just the next second of rotation.
class SafeRotationRPP : public
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
  using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    Base::configure(parent, name, tf, costmap_ros);
    turn_limited_pub_ = parent.lock()->create_publisher<std_msgs::msg::Bool>(
      "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  }

  void activate() override
  {
    Base::activate();
    turn_limited_pub_->on_activate();
    publishTurnLimit();
  }

  void deactivate() override
  {
    turn_limited_pub_->on_deactivate();
    Base::deactivate();
  }

  void cleanup() override
  {
    turn_limited_pub_.reset();
    Base::cleanup();
  }

  void reset() override
  {
    turning_ = false;
    aligning_terminal_yaw_ = false;
    terminal_arrival_seen_ = false;
    terminal_slow_turn_ = false;
    rotation_blocked_ = false;
    has_reached_xy_tolerance_ = false;
    // Keep a tripped turn guard across FollowPath retries for the same goal.
    Base::reset();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    std::vector<TrackingPose> poses;
    for (const auto & p : path.poses) {
      poses.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
    }
    const bool reverse = shortReversePath(poses);
    if (reverse != short_reverse_plan_) {turning_ = false;}
    short_reverse_plan_ = reverse;
    bool new_goal = false;
    if (!path.poses.empty()) {
      const auto & goal = path.poses.back().pose;
      const double goal_yaw = tf2::getYaw(goal.orientation);
      if (!have_goal_ || path.header.frame_id != goal_frame_ ||
        std::hypot(goal.position.x - goal_x_, goal.position.y - goal_y_) > 0.03 ||
        std::abs(wrapAngle(goal_yaw - goal_yaw_)) > 0.05)
      {
        new_goal = true;
        turn_guard_.reset();
        turn_limit_exceeded_ = false;
        publishTurnLimit();
        goal_frame_ = path.header.frame_id;
        goal_x_ = goal.position.x;
        goal_y_ = goal.position.y;
        goal_yaw_ = goal_yaw;
        have_goal_ = true;
      }
    }
    // Replanning every two seconds must not interrupt and restart an
    // already-safe turn toward the same goal. That repeatedly changed the
    // heading target and led to back-and-forth turns near arrival.
    if (new_goal) {
      turning_ = false;
      aligning_terminal_yaw_ = false;
      terminal_arrival_seen_ = false;
      terminal_slow_turn_ = false;
      has_reached_xy_tolerance_ = false;
    }
    // A blocked sweep may be retried only after a fresh global plan.
    rotation_blocked_ = false;
    Base::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    if (cancelling_) {
      turning_ = false;
      return Base::computeVelocityCommands(pose, velocity, goal_checker);
    }
    // A single clear costmap update must not restart an already-invalidated
    // turn and reset controller patience indefinitely. Require a new plan.
    if (rotation_blocked_) {
      throw nav2_core::NoValidControl("Rotation blocked on this plan; waiting for a new plan");
    }
    std::unique_lock<std::mutex> parameter_lock(param_handler_->getMutex());
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const bool previously_limited = turn_limit_exceeded_;
    turn_limit_exceeded_ = turn_guard_.update(
      pose.pose.position.x, pose.pose.position.y, yaw);
    if (previously_limited != turn_limit_exceeded_) {publishTurnLimit();}
    if (turn_limit_exceeded_) {
      turning_ = false;
      turn_limit_exceeded_ = true;
      RCLCPP_WARN(logger_,
        "Turn limit at local pose (%.3f, %.3f, %.3f), target goal (%.3f, %.3f, %.3f), "
        "remaining turn %.3f, terminal alignment %d",
        pose.pose.position.x, pose.pose.position.y, yaw, goal_x_, goal_y_, goal_yaw_,
        remaining_, aligning_terminal_yaw_);
      throw nav2_core::NoValidControl(
              "Stationary turn budget exceeded; a translated escape is required");
    }
    auto plan = path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
    if (plan.poses.empty()) {
      throw nav2_core::InvalidPath("Empty transformed path");
    }
    global_path_pub_->publish(plan);
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    double xy_tolerance = 0.05;
    double yaw_tolerance = 0.0872664626;
    if (goal_checker && goal_checker->getTolerances(tolerance, velocity_tolerance)) {
      xy_tolerance = tolerance.position.x;
      yaw_tolerance = std::abs(tf2::getYaw(tolerance.orientation));
    }
    goal_dist_tol_ = xy_tolerance;
    std::vector<TrackingPose> tracking_poses;
    for (const auto & p : plan.poses) {
      tracking_poses.push_back({p.pose.position.x, p.pose.position.y,
          tf2::getYaw(p.pose.orientation)});
    }
    const auto target = trackingTarget(tracking_poses);
    double angle = target.at_turn ? target.planned_turn :
      wrapAngle(target.initial_angle + (short_reverse_plan_ ? kPi : 0.0));
    const auto & end = plan.poses.back().pose;
    double remaining_length = 0.0;
    for (size_t i = 1; i < plan.poses.size(); ++i) {
      const auto & a = plan.poses[i - 1].pose.position;
      const auto & b = plan.poses[i].pose.position;
      remaining_length += std::hypot(b.x - a.x, b.y - a.y);
    }
    // A route leaving a nearby goal to align must finish its travel before
    // latching final yaw; proximity at its launch is not terminal arrival.
    const double goal_yaw_error = wrapAngle(tf2::getYaw(end.orientation));
    const double goal_xy_error = std::hypot(end.position.x, end.position.y);
    // A straight route may enter the 5 cm XY disk where the final pivot is
    // blocked, yet have a clear last few centimetres and a clear pivot at
    // the exact endpoint. Keep tracking that short line before turning.
    const bool defer_terminal_pivot = remaining_length <= 0.15 &&
      goal_xy_error < xy_tolerance && goal_xy_error > 0.015 &&
      end.position.x > 0.015 && std::abs(end.position.y) < 0.9 * xy_tolerance &&
      std::abs(goal_yaw_error) > yaw_tolerance &&
      !chooseRotation(goal_yaw_error,
        [&](double sweep) {return sweepClear(pose, yaw, sweep);});
    if (defer_terminal_pivot) {
      turning_ = false;
      aligning_terminal_yaw_ = false;
    }
    const bool at_goal = remaining_length <= 0.15 &&
      goal_xy_error < xy_tolerance && !defer_terminal_pivot;
    // Match the non-stateful goal checker: real or localization drift beyond
    // XY tolerance during alignment resumes endpoint tracking.
    has_reached_xy_tolerance_ = at_goal;
    if (at_goal && !terminal_arrival_seen_) {
      terminal_slow_turn_ = std::abs(goal_yaw_error) <= 1.8;
    }
    terminal_arrival_seen_ = terminal_arrival_seen_ || at_goal;
    if (at_goal && std::abs(goal_yaw_error) > yaw_tolerance) {
      aligning_terminal_yaw_ = true;
    }
    if (aligning_terminal_yaw_ && std::abs(goal_yaw_error) <= yaw_tolerance) {
      aligning_terminal_yaw_ = false;
      turning_ = false;
    }
    if (aligning_terminal_yaw_ && std::hypot(end.position.x, end.position.y) > 0.15) {
      aligning_terminal_yaw_ = false;
      turning_ = false;
      throw nav2_core::NoValidControl("Final pivot drift exceeds checked terminal region");
    }
    if (!at_goal && remaining_length <= 0.15) {
      // After skid/localization drift during the final pivot, the old line
      // tangent can point away from the goal. Rejoin the exact endpoint using
      // its current bearing instead of repeatedly rotating toward that tangent.
      angle = wrapAngle(std::atan2(end.position.y, end.position.x) +
        (short_reverse_plan_ ? kPi : 0.0));
    }
    if (has_reached_xy_tolerance_ || aligning_terminal_yaw_) {
      angle = goal_yaw_error;
      // Keep a validated long sweep committed until it finishes. Resetting
      // here on every control tick would repeatedly choose the blocked short
      // direction instead of continuing through the clear side.
    }
    if (!aligning_terminal_yaw_ && terminalReverseCorrectionAllowed(
        end.position.x, end.position.y, remaining_length, xy_tolerance,
        goal_yaw_error, terminal_arrival_seen_))
    {
      turning_ = false;
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      // Stop the pivot before retreating. Check the whole straight correction
      // with the physical footprint, in addition to native time-to-collision.
      if (std::abs(velocity.angular.z) > 0.02 || velocity.linear.x > 0.02) {return command;}
      const double retreat = -end.position.x;
      const int samples = std::max(1, static_cast<int>(std::ceil(retreat / 0.01)));
      for (int i = 0; i <= samples; ++i) {
        const double d = retreat * static_cast<double>(i) / samples;
        if (collision_checker_->inCollision(pose.pose.position.x - d * std::cos(yaw),
            pose.pose.position.y - d * std::sin(yaw), yaw))
        {
          throw nav2_core::NoValidControl("Terminal reverse correction corridor blocked");
        }
      }
      command.twist.linear.x = -std::min(0.08, params_->desired_linear_vel);
      if (collision_checker_->isCollisionImminent(
          pose, command.twist.linear.x, 0.0, retreat))
      {
        throw nav2_core::NoValidControl("Terminal reverse correction collision check failed");
      }
      return command;
    }
    const bool continue_terminal_tracking = !target.at_turn &&
      canApproachGoalWithoutStationaryTurn(
      short_reverse_plan_ ? -end.position.x : end.position.x,
      short_reverse_plan_ ? -end.position.y : end.position.y, remaining_length, xy_tolerance);
    const bool goal_rotation = has_reached_xy_tolerance_ || aligning_terminal_yaw_;
    if (remaining_length <= 0.15) {
      RCLCPP_DEBUG_THROTTLE(logger_, *costmap_ros_->get_clock(), 1000,
        "terminal xy=(%.3f,%.3f) yaw=%.3f remaining=%.3f at=%d align=%d turning=%d angle=%.3f error=%.3f",
        end.position.x, end.position.y, yaw, remaining_length, at_goal,
        aligning_terminal_yaw_, turning_, angle, goal_yaw_error);
    }
    const double threshold = goal_rotation ? yaw_tolerance :
      (target.at_turn ? 0.04 : (continue_terminal_tracking ? 0.80 :
      params_->rotate_to_heading_min_angle));
    if (!turning_) {
      if (std::abs(angle) > threshold) {
        const auto chosen = chooseRotation(angle,
            [&](double sweep) {return sweepClear(pose, yaw, sweep);},
          goal_rotation || std::hypot(end.position.x, end.position.y) > 0.50);
        if (!chosen) {
          rotation_blocked_ = true;
          throw nav2_core::NoValidControl("Both complete heading sweeps blocked; replan required");
        }
        if (std::abs(*chosen) > StationaryTurnGuard::kMaxSweep) {
          turn_limit_exceeded_ = true;
          publishTurnLimit();
          throw nav2_core::NoValidControl("Safe heading sweep exceeds stationary turn limit");
        }
        remaining_ = *chosen;
        turn_tolerance_ = goal_rotation ? yaw_tolerance : 0.04;
        turning_ = true;
        last_yaw_ = yaw;
      }
    } else {
      remaining_ -= wrapAngle(yaw - last_yaw_);
      last_yaw_ = yaw;
    }

    if (turning_) {
      std_msgs::msg::Bool rotating;
      rotating.data = true;
      is_rotating_to_heading_pub_->publish(rotating);
      if (std::abs(remaining_) < turn_tolerance_) {
        turning_ = false;
        // Stop before handing translation back to RPP.
        geometry_msgs::msg::TwistStamped stop;
        stop.header = pose.header;
        return stop;
      }
      if (!sweepClear(pose, yaw, remaining_)) {
        // Do not silently change direction when fresh observations invalidate
        // the committed sweep. Stop and let planning choose another route.
        turning_ = false;
        rotation_blocked_ = true;
        throw nav2_core::NoValidControl("Committed rotation invalidated by updated costmap");
      }
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      // Brake before a change of sign; acceleration limiting must never
      // turn a small requested right correction into another left command.
      if (remaining_ * velocity.angular.z < 0.0 && std::abs(velocity.angular.z) > 0.02) {
        return command;
      }
      // Near a terminal pose, the skid-steer chassis drifts laterally during
      // a pivot. Increase the final correction modestly while keeping it
      // below the normal heading-alignment rate.
      constexpr double kMinimumLoadedAngularSpeed = 0.22;
      constexpr double kTerminalAngularLimit = 0.24;
      const bool terminal_turn = terminal_slow_turn_ && remaining_length <= 0.25 &&
        std::hypot(end.position.x, end.position.y) <= 0.25;
      const double angular_limit = terminal_turn ?
        std::min(params_->rotate_to_heading_angular_vel, kTerminalAngularLimit) :
        params_->rotate_to_heading_angular_vel;
      double angular = std::copysign(
        std::min(angular_limit,
        std::sqrt(2.0 * params_->max_angular_accel * std::abs(remaining_))), remaining_);
      angular = std::clamp(angular,
        velocity.angular.z - params_->max_angular_accel * control_duration_,
        velocity.angular.z + params_->max_angular_accel * control_duration_);
      // Keep an exact zero at completion and a reliable active-turn floor.
      angular = std::copysign(
        std::max(std::abs(angular), kMinimumLoadedAngularSpeed), angular);
      if (collision_checker_->isCollisionImminent(pose, 0.0, angular, 0.0)) {
        rotation_blocked_ = true;
        throw nav2_core::NoValidControl("Rotation command collision check failed");
      }
      command.twist.angular.z = angular;
      return command;
    }
    if (has_reached_xy_tolerance_) {
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
    if ((short_reverse_plan_ && velocity.linear.x > .02) ||
      (!short_reverse_plan_ && velocity.linear.x < -.02)) {
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
    // Retain RPP's speed regulation, but follow the immediate segment rather
    // than letting Base demand an in-place turn toward a far-away carrot.
    // Stop lookahead at a planned rotate-in-place primitive. If the longer
    // tracking arc collides, try shorter carrots on this same validated path.
    std_msgs::msg::Bool rotating;
    rotating.data = false;
    is_rotating_to_heading_pub_->publish(rotating);
    double lookahead = std::min(getLookAheadDistance(velocity), target.lookahead_limit);
    lookahead = std::max(0.015, lookahead);
    while (true) {
      const auto carrot = getLookAheadPoint(lookahead, plan);
      const auto & p = carrot.pose.position;
      geometry_msgs::msg::PointStamped carrot_marker;
      carrot_marker.header = carrot.header;
      carrot_marker.point = p;
      carrot_pub_->publish(carrot_marker);
      const double d2 = p.x * p.x + p.y * p.y;
      const double curvature = d2 > 0.0001 ? 2.0 * p.y / d2 : 0.0;
      double linear = params_->desired_linear_vel;
      double sign = 1.0;
      applyConstraints(curvature, velocity,
        collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y),
        plan, linear, sign);
      if (short_reverse_plan_) {linear = -std::min(linear, 0.08);}
      const double angular = linear * curvature;
      const double carrot_distance = std::hypot(p.x, p.y);
      if (!params_->use_collision_detection ||
        !collision_checker_->isCollisionImminent(pose, linear, angular, carrot_distance))
      {
        geometry_msgs::msg::TwistStamped command;
        command.header = pose.header;
        command.twist.linear.x = linear;
        command.twist.angular.z = angular;
        return command;
      }
      if (lookahead <= 0.05) {break;}
      lookahead = std::max(0.05, lookahead * 0.5);
    }
    throw nav2_core::NoValidControl("No collision-free tracking arc on the immediate path");
  }

private:
  void publishTurnLimit()
  {
    if (!turn_limited_pub_ || !turn_limited_pub_->is_activated()) {return;}
    std_msgs::msg::Bool state;
    state.data = turn_limit_exceeded_;
    turn_limited_pub_->publish(state);
  }
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr turn_limited_pub_;
  bool sweepClear(const geometry_msgs::msg::PoseStamped & pose, double yaw, double angle)
  {
    return rotationSweepClear(yaw, angle, [&](double heading) {
               return collision_checker_->inCollision(
        pose.pose.position.x, pose.pose.position.y, heading);
    });
  }
  bool turning_{false};
  bool short_reverse_plan_{false};
  bool aligning_terminal_yaw_{false};
  bool terminal_arrival_seen_{false};
  bool terminal_slow_turn_{false};
  bool rotation_blocked_{false};
  bool turn_limit_exceeded_{false};
  bool have_goal_{false};
  std::string goal_frame_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  StationaryTurnGuard turn_guard_;
  double remaining_{0.0};
  double turn_tolerance_{0.04};
  double last_yaw_{0.0};
};
}  // namespace tai_robot_one

PLUGINLIB_EXPORT_CLASS(tai_robot_one::SafeRotationRPP, nav2_core::Controller)
