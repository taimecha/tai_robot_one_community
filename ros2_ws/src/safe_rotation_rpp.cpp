// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tai_robot_one/rotation_sweep.hpp"
#include "tai_robot_one/path_tracking.hpp"
#include "tai_robot_one/terminal_braking.hpp"

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
    auto node = parent.lock();
    auto parameter = [&](const std::string & key, const rclcpp::ParameterValue & value) {
        if (!node->has_parameter(name + "." + key)) {node->declare_parameter(name + "." + key, value);}
      };
    parameter("predictive_terminal_braking", rclcpp::ParameterValue(false));
    parameter("terminal_command_latency", rclcpp::ParameterValue(0.20));
    parameter("terminal_angular_deceleration", rclcpp::ParameterValue(1.5));
    parameter("allow_short_rear_goal_correction", rclcpp::ParameterValue(false));
    parameter("allow_reverse_route_tracking", rclcpp::ParameterValue(true));
    parameter("terminal_reverse_speed", rclcpp::ParameterValue(0.08));
    parameter("transient_tf_hold_timeout", rclcpp::ParameterValue(0.0));
    node->get_parameter(name + ".predictive_terminal_braking", predictive_terminal_braking_);
    node->get_parameter(name + ".terminal_command_latency", terminal_command_latency_);
    node->get_parameter(name + ".terminal_angular_deceleration", terminal_angular_deceleration_);
    node->get_parameter(name + ".allow_short_rear_goal_correction", allow_short_rear_goal_correction_);
    node->get_parameter(name + ".allow_reverse_route_tracking", allow_reverse_route_tracking_);
    node->get_parameter(name + ".terminal_reverse_speed", terminal_reverse_speed_);
    node->get_parameter(name + ".transient_tf_hold_timeout", transient_tf_hold_timeout_);
    if (!std::isfinite(terminal_command_latency_) || terminal_command_latency_ < 0.0 ||
      !std::isfinite(terminal_angular_deceleration_) || terminal_angular_deceleration_ <= 0.0 ||
      !std::isfinite(terminal_reverse_speed_) || terminal_reverse_speed_ <= 0.0 || terminal_reverse_speed_ > .10)
    {
      throw std::invalid_argument("Invalid terminal braking or bounded reverse parameters");
    }
    if (!std::isfinite(transient_tf_hold_timeout_) ||
      transient_tf_hold_timeout_ < 0.0 || transient_tf_hold_timeout_ > 1.0)
    {
      throw std::invalid_argument("Transient TF hold must be between zero and one second");
    }
    RCLCPP_INFO(logger_, "Terminal control: predictive braking=%d, latency budget=%.3f s, bounded rear goal=%d",
      predictive_terminal_braking_, terminal_command_latency_, allow_short_rear_goal_correction_);
    turn_limited_pub_ = parent.lock()->create_publisher<std_msgs::msg::Bool>(
      "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  }

  void activate() override
  {
    tf_hold_started_.reset();
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
    terminal_braking_ = false;
    terminal_stop_ticks_ = 0;
    rotation_blocked_ = false;
    goal_sweep_flipped_ = false;
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
    const bool reverse = allow_reverse_route_tracking_ && shortReversePath(poses);
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
      tf_hold_started_.reset();
      turning_ = false;
      aligning_terminal_yaw_ = false;
      terminal_arrival_seen_ = false;
      terminal_slow_turn_ = false;
      terminal_braking_ = false;
      terminal_stop_ticks_ = 0;
      goal_sweep_flipped_ = false;
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
    const auto started = std::chrono::steady_clock::now();
    try {
      auto command = computeWithCurrentTransforms(pose, velocity, goal_checker);
      if (tf_hold_started_) {
        RCLCPP_INFO(logger_, "TF recovered; continuing the same navigation goal");
        tf_hold_started_.reset();
      }
      return command;
    } catch (const nav2_core::ControllerTFError & error) {
      if (transient_tf_hold_timeout_ <= 0.0 || cancelling_) {throw;}
      if (!tf_hold_started_) {
        tf_hold_started_ = started;
        RCLCPP_WARN(logger_, "TF temporarily unavailable; commanding zero for at most %.2f s: %s",
          transient_tf_hold_timeout_, error.what());
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *tf_hold_started_).count();
      if (elapsed >= transient_tf_hold_timeout_) {throw;}
      // Never reuse the preceding moving command, restamp a pose, extrapolate
      // TF, or bypass footprint checks. Only a fully successful next compute
      // may release this hold. Same-goal replans/reset do not reset its budget.
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
  }

private:
  geometry_msgs::msg::TwistStamped computeWithCurrentTransforms(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
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
    // A straight route may enter the XY disk while still several centimetres
    // short of the goal center. Starting a large skid-steer pivot at the outer
    // rim of xy_tolerance lets a few centimetres of pivot drift push the robot
    // back outside xy_tolerance and cause back-and-forth turns. Finish
    // advancing near the goal center when it lies straight ahead.
    const bool center_approach_defer = !aligning_terminal_yaw_ && !turning_ &&
      !short_reverse_plan_ && remaining_length <= 0.15 &&
      goal_xy_error < xy_tolerance && end.position.x > 0.025 &&
      std::abs(end.position.y) < 0.65 * xy_tolerance &&
      std::abs(goal_yaw_error) > yaw_tolerance;
    // A straight route may also enter the XY disk where the final pivot is
    // blocked, yet have a clear last few centimetres and a clear pivot at
    // the exact endpoint. Keep tracking that short line before turning.
    const bool blocked_pivot_defer = remaining_length <= 0.15 &&
      goal_xy_error < xy_tolerance && goal_xy_error > 0.015 &&
      end.position.x > 0.015 && std::abs(end.position.y) < 0.9 * xy_tolerance &&
      std::abs(goal_yaw_error) > yaw_tolerance &&
      !chooseRotation(goal_yaw_error,
        [&](double sweep) {return sweepClear(pose, yaw, sweep);});
    const bool defer_terminal_pivot = center_approach_defer || blocked_pivot_defer;
    if (defer_terminal_pivot) {
      turning_ = false;
      aligning_terminal_yaw_ = false;
    }
    // During a route-heading turn toward the goal (e.g. after pivot drift
    // pushed dist > xy_tolerance), require an inner XY margin before switching
    // back to goal_yaw so skid-steer wobble across xy_tolerance cannot trigger
    // an endless ping-pong oscillation.
    const double xy_entry_tolerance = (turning_ && !turning_to_goal_) ?
      std::min(xy_tolerance, std::max(0.025, 0.55 * xy_tolerance)) : xy_tolerance;
    const bool at_goal = remaining_length <= 0.15 &&
      goal_xy_error < xy_entry_tolerance && !defer_terminal_pivot;
    // Match the non-stateful goal checker: real or localization drift beyond
    // XY tolerance during alignment resumes endpoint tracking.
    has_reached_xy_tolerance_ = at_goal;
    if (at_goal && !terminal_arrival_seen_) {
      terminal_slow_turn_ = std::abs(goal_yaw_error) <= 1.8;
    }
    terminal_arrival_seen_ = terminal_arrival_seen_ || at_goal;
    if (terminal_braking_) {
      // Keep zero through real braking and feedback delivery. Do not turn
      // back the other way on a still-moving chassis because one TF sample
      // has crossed the heading target.
      if (std::abs(velocity.angular.z) > .02 ||
        std::hypot(velocity.linear.x, velocity.linear.y) > .015)
      {
        terminal_stop_ticks_ = 0;
      } else {
        ++terminal_stop_ticks_;
      }
      if (terminal_stop_ticks_ < 4) {
        geometry_msgs::msg::TwistStamped stop;
        stop.header = pose.header;
        return stop;
      }
      terminal_braking_ = false;
      turning_ = false;
    }
    if (at_goal && std::abs(goal_yaw_error) > yaw_tolerance) {
      aligning_terminal_yaw_ = true;
    }
    if (aligning_terminal_yaw_ && std::abs(goal_yaw_error) <= yaw_tolerance) {
      aligning_terminal_yaw_ = false;
      turning_ = false;
      terminal_braking_ = true;
      terminal_stop_ticks_ = 0;
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
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
        goal_yaw_error, terminal_arrival_seen_, allow_short_rear_goal_correction_))
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
      command.twist.linear.x = -std::min(terminal_reverse_speed_, params_->desired_linear_vel);
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
    if (turning_ && goal_rotation != turning_to_goal_) {
      // Entering/leaving the XY disk changes the requested heading from the
      // route bearing to the saved goal yaw. Never keep integrating the old
      // remaining angle under the new role: stop, then select a fresh sweep.
      turning_ = false;
      terminal_braking_ = true;
      terminal_stop_ticks_ = 0;
      RCLCPP_INFO(logger_, "Stopping before turn reference changes from %s to %s",
        turning_to_goal_ ? "goal yaw" : "route heading",
        goal_rotation ? "goal yaw" : "route heading");
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
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
        const bool allow_long = (goal_rotation || std::hypot(end.position.x, end.position.y) > 0.50)
          && std::abs(wrapAngle(angle)) >= 1.75;
        const auto chosen = chooseRotation(angle,
            [&](double sweep) {return sweepClear(pose, yaw, sweep);},
          allow_long);
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
        turning_to_goal_ = goal_rotation;
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
        if (goal_rotation) {
          terminal_braking_ = true;
          terminal_stop_ticks_ = 0;
        }
        // Stop before handing translation back to RPP.
        geometry_msgs::msg::TwistStamped stop;
        stop.header = pose.header;
        return stop;
      }
      if (!sweepClear(pose, yaw, remaining_)) {
        const bool allow_opposite = (goal_rotation || std::hypot(end.position.x, end.position.y) > 0.50)
          && std::abs(wrapAngle(remaining_)) >= 1.75;
        if (allow_opposite && !goal_sweep_flipped_) {
          const double opposite = remaining_ > 0.0 ? remaining_ - 2.0 * kPi : remaining_ + 2.0 * kPi;
          if (std::abs(opposite) <= StationaryTurnGuard::kMaxSweep && sweepClear(pose, yaw, opposite)) {
            goal_sweep_flipped_ = true;
            turning_ = false;
            terminal_braking_ = true;
            terminal_stop_ticks_ = 0;
            RCLCPP_WARN(logger_,
              "Committed sweep blocked by updated costmap; braking to switch once to clear opposite sweep (%.1f deg)",
              opposite * 180.0 / kPi);
            geometry_msgs::msg::TwistStamped stop;
            stop.header = pose.header;
            return stop;
          }
        }
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
      const bool terminal_turn = (terminal_slow_turn_ || predictive_terminal_braking_) && remaining_length <= 0.25 &&
        std::hypot(end.position.x, end.position.y) <= 0.25;
      if (predictive_terminal_braking_ && terminal_turn &&
        shouldBrakeTerminalTurn(remaining_, velocity.angular.z,
          terminal_command_latency_, terminal_angular_deceleration_))
      {
        terminal_braking_ = true;
        terminal_stop_ticks_ = 0;
        RCLCPP_INFO(logger_, "Braking terminal turn early: remaining %.2f deg, measured %.3f rad/s",
          remaining_ * 180.0 / kPi, velocity.angular.z);
        return command;
      }
      // Only taper down to the slow terminal angular cap in the final ~25 deg
      // so large 90-180 deg pivots at A/D turn crisply without skid-steer stiction walk.
      const bool fine_terminal_turn = terminal_turn && std::abs(remaining_) <= 0.45;
      double angular_limit = fine_terminal_turn ?
        std::min(params_->rotate_to_heading_angular_vel, kTerminalAngularLimit) :
        params_->rotate_to_heading_angular_vel;
      if (predictive_terminal_braking_ && fine_terminal_turn) {
        angular_limit = std::min(angular_limit, kMinimumLoadedAngularSpeed);
      }
      const double angular = boundedLoadedTurnCommand(remaining_, velocity.angular.z,
        angular_limit, kMinimumLoadedAngularSpeed, params_->max_angular_accel, control_duration_);
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
  bool turning_to_goal_{false};
  bool short_reverse_plan_{false};
  bool aligning_terminal_yaw_{false};
  bool terminal_arrival_seen_{false};
  bool terminal_slow_turn_{false};
  bool predictive_terminal_braking_{false};
  bool terminal_braking_{false};
  int terminal_stop_ticks_{0};
  double terminal_command_latency_{0.20};
  double terminal_angular_deceleration_{1.5};
  bool allow_short_rear_goal_correction_{false};
  bool allow_reverse_route_tracking_{true};
  double terminal_reverse_speed_{0.08};
  double transient_tf_hold_timeout_{0.0};
  std::optional<std::chrono::steady_clock::time_point> tf_hold_started_;
  bool rotation_blocked_{false};
  bool goal_sweep_flipped_{false};
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
