// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include "tai_robot_one/path_clearance.hpp"
#include "tai_robot_one/path_selection.hpp"
#include "tai_robot_one/escape_memory.hpp"

namespace tai_robot_one
{
// Replace a lattice stop-turn-go corner with a tangent circular fillet only
// when the complete moving footprint is clear. Keep the original cusp if the
// bend, neighboring straight segments, or available clearance cannot fit it.
template<class Collision>
size_t roundCheckedLatticeCorners(nav_msgs::msg::Path & path, double resolution,
  Collision collision)
{
  if (path.poses.size() < 5) {return 0;}
  const auto xy = [](const geometry_msgs::msg::PoseStamped & p) {
      return std::pair<double, double>{p.pose.position.x, p.pose.position.y};
    };
  const auto dist = [&](size_t a, size_t b) {
      const auto [ax, ay] = xy(path.poses[a]);
      const auto [bx, by] = xy(path.poses[b]);
      return std::hypot(bx - ax, by - ay);
    };
  auto make_pose = [&](size_t reference, double px, double py, double yaw) {
      auto p = path.poses[reference];
      p.pose.position.x = px;
      p.pose.position.y = py;
      p.pose.orientation.x = p.pose.orientation.y = 0.0;
      p.pose.orientation.z = std::sin(yaw / 2.0);
      p.pose.orientation.w = std::cos(yaw / 2.0);
      return p;
    };
  size_t rounded = 0;
  for (size_t i = 1; i + 1 < path.poses.size();) {
    const double first_turn = wrapAngle(
      tf2::getYaw(path.poses[i].pose.orientation) -
      tf2::getYaw(path.poses[i - 1].pose.orientation));
    if (dist(i - 1, i) >= .005 || std::abs(first_turn) <= .02) {++i; continue;}
    const size_t pivot = i - 1;
    size_t end = i;
    while (end + 1 < path.poses.size() && dist(pivot, end + 1) < .005) {++end;}
    const auto [cx, cy] = xy(path.poses[pivot]);
    if (!std::isfinite(cx) || !std::isfinite(cy) ||
      dist(pivot, path.poses.size() - 1) < .30)
    {i = end + 1; continue;}
    size_t before = pivot;
    while (before > 0 && dist(before - 1, pivot) < .02) {--before;}
    size_t after = end + 1;
    while (after + 1 < path.poses.size() && dist(pivot, after) < .02) {++after;}
    if (before == 0 || after >= path.poses.size()) {i = end + 1; continue;}
    const auto [bx, by] = xy(path.poses[before - 1]);
    const auto [fx, fy] = xy(path.poses[after]);
    const double incoming = std::atan2(cy - by, cx - bx);
    const double outgoing = std::atan2(fy - cy, fx - cx);
    const double turn = wrapAngle(outgoing - incoming);
    const double planned_turn = wrapAngle(
      tf2::getYaw(path.poses[end].pose.orientation) -
      tf2::getYaw(path.poses[pivot].pose.orientation));
    if (!std::isfinite(incoming) || !std::isfinite(outgoing) ||
      !std::isfinite(planned_turn) ||
      std::abs(turn) < .08 || std::abs(turn) > .70 ||
      std::abs(wrapAngle(turn - planned_turn)) > .15 ||
      std::abs(wrapAngle(incoming - tf2::getYaw(path.poses[pivot].pose.orientation))) > .15 ||
      std::abs(wrapAngle(outgoing - tf2::getYaw(path.poses[end].pose.orientation))) > .15)
    {i = end + 1; continue;}
    bool accepted = false;
    for (double radius : {.40, .30}) {
      const double trim = radius * std::tan(std::abs(turn) / 2.0);
      if (trim < .025 || trim > .22) {continue;}
      size_t left = pivot;
      while (left > 0 && dist(left - 1, pivot) < trim - 1e-6) {--left;}
      size_t right = end + 1;
      while (right + 1 < path.poses.size() && dist(pivot, right) < trim - 1e-6) {
        ++right;
      }
      if (left == 0 || right >= path.poses.size() ||
        dist(left - 1, pivot) + 1e-6 < trim ||
        dist(pivot, right) + 1e-6 < trim)
      {continue;}
      const double ax = cx - trim * std::cos(incoming);
      const double ay = cy - trim * std::sin(incoming);
      const auto [lx, ly] = xy(path.poses[left - 1]);
      const auto [rx, ry] = xy(path.poses[right]);
      const double left_offset = std::abs(
        -(lx - cx) * std::sin(incoming) + (ly - cy) * std::cos(incoming));
      const double right_offset = std::abs(
        -(rx - cx) * std::sin(outgoing) + (ry - cy) * std::cos(outgoing));
      if (left_offset > .02 || right_offset > .02)
      {continue;}
      const double sign = std::copysign(1.0, turn);
      const double ox = ax - sign * radius * std::sin(incoming);
      const double oy = ay + sign * radius * std::cos(incoming);
      const double start_angle = std::atan2(ay - oy, ax - ox);
      const int samples = std::max(2, static_cast<int>(std::ceil(
        radius * std::abs(turn) / std::min(.0125, resolution / 2.0))));
      std::vector<geometry_msgs::msg::PoseStamped> arc;
      arc.reserve(samples + 1);
      bool clear = true;
      for (int k = 0; k <= samples; ++k) {
        const double t = static_cast<double>(k) / samples;
        const double a = start_angle + turn * t;
        auto q = make_pose(pivot, ox + radius * std::cos(a),
          oy + radius * std::sin(a), incoming + turn * t);
        if (collision(q.pose.position.x, q.pose.position.y,
            tf2::getYaw(q.pose.orientation)))
        {clear = false; break;}
        if (!arc.empty()) {
          const auto & prev = arc.back().pose;
          if (!poseSegmentClear(prev.position.x, prev.position.y,
              tf2::getYaw(prev.orientation), q.pose.position.x, q.pose.position.y,
              tf2::getYaw(q.pose.orientation), resolution, collision))
          {clear = false; break;}
        }
        arc.push_back(std::move(q));
      }
      if (!clear) {continue;}
      const auto & left_pose = path.poses[left - 1].pose;
      const auto & right_pose = path.poses[right].pose;
      const auto & first = arc.front().pose;
      const auto & last = arc.back().pose;
      if (!poseSegmentClear(left_pose.position.x, left_pose.position.y,
          tf2::getYaw(left_pose.orientation), first.position.x, first.position.y,
          tf2::getYaw(first.orientation), resolution, collision) ||
        !poseSegmentClear(last.position.x, last.position.y,
          tf2::getYaw(last.orientation), right_pose.position.x, right_pose.position.y,
          tf2::getYaw(right_pose.orientation), resolution, collision))
      {continue;}
      auto replacement = path.poses;
      replacement.erase(replacement.begin() + left, replacement.begin() + right);
      replacement.insert(replacement.begin() + left, arc.begin(), arc.end());
      path.poses = std::move(replacement);
      ++rounded;
      i = left + arc.size();
      accepted = true;
      break;
    }
    if (!accepted) {i = end + 1;}
  }
  return rounded;
}

// One subscription/executor cache per tree, shared by all geometry queries.
// This also prevents an unvisited recovery condition starting with an empty
// private cache while other conditions already have valid observations.
struct ClearanceData
{
  explicit ClearanceData(const rclcpp::Node::SharedPtr & parent)
  {
    static std::atomic<unsigned> sequence{0};
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter(
      "use_sim_time", parent->get_parameter("use_sim_time").as_bool())});
    node = std::make_shared<rclcpp::Node>(
      "footprint_route_check_" + std::to_string(sequence++), options);
    executor.add_node(node);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    selected_route_pub = node->create_publisher<nav_msgs::msg::Path>("/plan_selected", qos);
    turn_limit_sub = node->create_subscription<std_msgs::msg::Bool>(
      "/navigation/stationary_turn_limited", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != turn_limited) {turn_limit_anchor_valid = false;}
        turn_limited = msg->data;
      });
    global_sub = node->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", qos,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {global = msg;});
    local_sub = node->create_subscription<nav2_msgs::msg::Costmap>(
      "/local_costmap/costmap_raw", qos,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {local = msg;});
    footprint_sub = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/local_costmap/published_footprint", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {footprint = msg;});
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_route_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turn_limit_sub;
  bool turn_limited{false}, turn_limit_anchor_valid{false};
  double turn_limit_odom_x{0.0}, turn_limit_odom_y{0.0};
  geometry_msgs::msg::PoseStamped route_goal;
  bool route_goal_valid{false};
  geometry_msgs::msg::PoseStamped completed_goal;
  bool completed_goal_valid{false};
  bool turnTranslationRequired(double odom_x, double odom_y)
  {
    if (!turn_limited) {return false;}
    if (!turn_limit_anchor_valid) {
      turn_limit_odom_x = odom_x;
      turn_limit_odom_y = odom_y;
      turn_limit_anchor_valid = true;
    }
    return std::hypot(odom_x - turn_limit_odom_x, odom_y - turn_limit_odom_y) <
           StationaryTurnGuard::kResetTranslation;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  nav2_msgs::msg::Costmap::ConstSharedPtr global, local;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
  // Shared across recovery conditions, but not carried into a new goal.
  bool turn_collision{false};
  double collision_min_retreat{0.15};
  bool failed_forward{false};
  geometry_msgs::msg::PoseStamped collision_goal;
  double collision_x{0.0}, collision_y{0.0}, collision_yaw{0.0};
  unsigned motion_revision{0};
  // A geometric end to a real retreat permits ONE stopped replan. It is
  // not proof of a dead end, and must not become a stationary retry loop.
  bool retreat_active{false}, retreat_stopped{false};
  geometry_msgs::msg::PoseStamped retreat_goal;
  double retreat_x{0.0}, retreat_y{0.0}, retreat_yaw{0.0};
  bool near_goal_reverse_pending{false}, near_goal_reverse_done{false};
  bool terminal_escape_needed{false};
  double near_goal_reverse_x{0.0}, near_goal_reverse_y{0.0};
  double near_goal_reverse_yaw{0.0};
  bool forward_active{false}, forward_geometric_stop{false};
  geometry_msgs::msg::PoseStamped forward_goal;
  double forward_x{0.0}, forward_y{0.0}, forward_yaw{0.0}, forward_distance{0.0};
  struct TurnAttempt {double x, y, heading;};
  std::vector<TurnAttempt> turn_attempts;
  geometry_msgs::msg::PoseStamped turn_goal;
  EscapeMemory escape_memory;
  bool turn_geometric_stop{false}, selected_turn_goal_facing{false}, turn_handoff{false};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_sub, local_sub;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub;
};

// Validate a candidate using the actual padded footprint, not a point or
// circumscribed circle. No commands are issued by this BT condition.
class PathFootprintClear final : public BT::ConditionNode
{
public:
  PathFootprintClear(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    auto parent = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (!config.blackboard->get("tai_clearance_data", data_)) {
      data_ = std::make_shared<ClearanceData>(parent);
      config.blackboard->set("tai_clearance_data", data_);
    }
    node_ = data_->node;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<bool>("data_only", false, "Check sensor/TF readiness without checking a path"),
      BT::InputPort<std::string>("motion", "path",
          "path/direct/direct_arc/forward/turn/terminal_turn/goal_pose_reached/turn_collision/reverse_needed/escape_available/"
          "recovery_allowed/replan_after_retreat/forward_candidate/forward_route/forward_direct/"
          "goal_approach_candidate/approach_route/motion_fault_free/"
          "forward_begin/forward_safe/forward_complete/forward_collision/turn_begin/lost_exit/route_control_available/goal_completed"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("candidate_distance", 0.15, "Forward route search distance"),
      BT::InputPort<double>("candidate_angle", 0.0, "Selected checked recovery turn"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("candidate_start"),
      BT::InputPort<double>("local_check_distance", 1.0, "Radius requiring live local checks"),
      BT::InputPort<double>("goal_xy_tolerance", 0.05, "Match the shared goal checker XY tolerance"),
      BT::InputPort<double>("goal_yaw_tolerance", 0.0872664626,
        "Match the shared goal checker yaw tolerance"),
      BT::InputPort<uint16_t>("planner_error", uint16_t{0}, "Planner result code"),
      BT::InputPort<uint16_t>("controller_error", uint16_t{0}, "Controller result code"),
      BT::InputPort<uint16_t>("motion_error", uint16_t{0}, "Motion action result code"),
      BT::InputPort<bool>("local_blocked", false, "Route failure is near the robot"),
      BT::InputPort<std::string>("planner_id", "SE2Fallback",
          "GridBased or SE2Fallback path semantics"),
      BT::InputPort<bool>("prepared", false, "Path already converted from Smac2D cell corners"),
      BT::OutputPort<nav_msgs::msg::Path>("checked_path"),
      BT::OutputPort<std::string>("path_planner"),
      BT::OutputPort<bool>("blocked_near_robot"),
      BT::OutputPort<double>("forward_distance"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("forward_start"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_goal"),
      BT::OutputPort<double>("turn_angle")
    };
  }

  BT::NodeStatus tick() override
  {
    if (getInput<std::string>("motion").value() == "goal_completed") {
      geometry_msgs::msg::PoseStamped goal;
      if (!data_->completed_goal_valid || !getInput("goal", goal)) {
        return BT::NodeStatus::FAILURE;
      }
      const auto & done = data_->completed_goal;
      return goal.header.frame_id == done.header.frame_id &&
             std::hypot(goal.pose.position.x - done.pose.position.x,
             goal.pose.position.y - done.pose.position.y) < 1e-6 &&
             std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
             tf2::getYaw(done.pose.orientation))) < 1e-6 ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    data_->executor.spin_some();
    global_ = data_->global;
    local_ = data_->local;
    footprint_ = data_->footprint;
    try {
      if (!global_ || !local_ || !footprint_ ||
        !fresh(global_->header.stamp, 3.0) || !fresh(local_->header.stamp, 1.0) ||
        !fresh(footprint_->header.stamp, 1.0))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "Route check waiting for fresh costmaps and footprint");
        return BT::NodeStatus::FAILURE;
      }
      // AMCL's map->odom can be older while stationary. Use current odometry
      // composed with the latest localization, not the oldest common TF time.
      const auto odom_robot = tf_->lookupTransform(
        local_->header.frame_id, "base_footprint", tf2::TimePointZero);
      if (!fresh(odom_robot.header.stamp, 1.0)) {return BT::NodeStatus::FAILURE;}
      const auto odom_to_map = tf_->lookupTransform(
        global_->header.frame_id, local_->header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped odom_pose, map_pose;
      odom_pose.pose.position.x = odom_robot.transform.translation.x;
      odom_pose.pose.position.y = odom_robot.transform.translation.y;
      odom_pose.pose.orientation = odom_robot.transform.rotation;
      tf2::doTransform(odom_pose, map_pose, odom_to_map);
      geometry_msgs::msg::TransformStamped footprint_tf;
      try {
        footprint_tf = tf_->lookupTransform(
          "base_footprint", footprint_->header.frame_id,
          rclcpp::Time(footprint_->header.stamp));
      } catch (const tf2::ExtrapolationException &) {
        // The costmap can publish its footprint a control tick ahead of the
        // latest odometry TF. Rejecting a currently followed route for that
        // small scheduling gap causes a needless halt and fresh plan.
        const auto latest = tf_->lookupTransform(
          "base_footprint", footprint_->header.frame_id, tf2::TimePointZero);
        const double lead = (rclcpp::Time(footprint_->header.stamp) -
          rclcpp::Time(latest.header.stamp)).seconds();
        if (lead < 0.0 || lead > 0.15) {throw;}
        footprint_tf = latest;
      }
      nav2_costmap_2d::Footprint body;
      for (const auto & p : footprint_->polygon.points) {
        geometry_msgs::msg::PointStamped in, out;
        in.point.x = p.x;
        in.point.y = p.y;
        in.point.z = p.z;
        tf2::doTransform(in, out, footprint_tf);
        body.push_back(out.point);
      }
      if (body.size() < 3) {return BT::NodeStatus::FAILURE;}
      bool data_only = false;
      (void)getInput("data_only", data_only);
      if (data_only) {return BT::NodeStatus::SUCCESS;}
      const auto motion = getInput<std::string>("motion").value();
      const double goal_xy_tolerance = getInput<double>("goal_xy_tolerance").value();
      const double goal_yaw_tolerance = getInput<double>("goal_yaw_tolerance").value();
      if (!std::isfinite(goal_xy_tolerance) || goal_xy_tolerance < .01 ||
        goal_xy_tolerance > .20 || !std::isfinite(goal_yaw_tolerance) ||
        goal_yaw_tolerance < .02 || goal_yaw_tolerance > .35)
      {return BT::NodeStatus::FAILURE;}
      if (motion == "goal_pose_reached") {
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id) {
          return BT::NodeStatus::FAILURE;
        }
        return std::hypot(goal.pose.position.x - map_pose.pose.position.x,
                 goal.pose.position.y - map_pose.pose.position.y) <= goal_xy_tolerance &&
               std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
               tf2::getYaw(map_pose.pose.orientation))) <= goal_yaw_tolerance ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      if (motion == "route_control_available") {
        geometry_msgs::msg::PoseStamped goal;
        if (getInput("goal", goal)) {
          if (data_->route_goal_valid &&
            (goal.header.frame_id != data_->route_goal.header.frame_id ||
            std::hypot(goal.pose.position.x - data_->route_goal.pose.position.x,
            goal.pose.position.y - data_->route_goal.pose.position.y) > 1e-6 ||
            std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
            tf2::getYaw(data_->route_goal.pose.orientation))) > 1e-6))
          {
            // A new goal is allowed to reach setPlan(), which resets the
            // controller guard. A previous aborted goal must not lock it out.
            data_->turn_limited = false;
            data_->turn_limit_anchor_valid = false;
          }
          data_->route_goal = goal;
          data_->route_goal_valid = true;
        }
        const auto & odom = odom_robot.transform.translation;
        // FollowPath cannot clear its persistent turn guard without actual
        // translation. Route retries at the same pose are futile. Let the
        // checked DriveOnHeading/BackUp escape move before retrying paths.
        return data_->turnTranslationRequired(odom.x, odom.y) ?
          BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      }
      const bool forward_direct = motion == "forward_direct";
      const bool direct = motion == "direct" || forward_direct;
      const bool direct_arc = motion == "direct_arc";
      const bool geometric_direct = direct || direct_arc;
      const bool forward_route = motion == "forward_route" || forward_direct;
      const bool approach_route = motion == "approach_route";
      const bool path_query = motion == "path" || geometric_direct ||
        forward_route || approach_route;
      const double requested_range = getInput<double>("local_check_distance").value();
      if (!std::isfinite(requested_range) || requested_range < 1.0 || requested_range > 1.30) {
        return BT::NodeStatus::FAILURE;
      }
      double local_range = requested_range;
      bool reverse_path = getInput<std::string>("planner_id").value() == "DirectReverse";
      nav_msgs::msg::Path path;
      if ((motion == "path" || (forward_route && !forward_direct) || approach_route) &&
        (!getInput("path", path) || path.poses.empty() ||
        path.header.frame_id != global_->header.frame_id))
      {
        return BT::NodeStatus::FAILURE;
      }
      if (geometric_direct) {
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id) {
          return BT::NodeStatus::FAILURE;
        }
        auto departure_pose = map_pose;
        if (forward_direct && (!getInput("candidate_start", departure_pose) ||
          departure_pose.header.frame_id != global_->header.frame_id))
        {return BT::NodeStatus::FAILURE;}
        const auto & start = departure_pose.pose;
        const double dx = goal.pose.position.x - start.position.x;
        const double dy = goal.pose.position.y - start.position.y;
        const double distance = std::hypot(dx, dy);
        const double yaw = tf2::getYaw(start.orientation);
        const double longitudinal = dx * std::cos(yaw) + dy * std::sin(yaw);
        const double rear_bearing = wrapAngle(std::atan2(dy, dx) + kPi - yaw);
        reverse_path = direct && !forward_direct &&
          distance > goal_xy_tolerance && distance <= 1.0 + 1e-6 &&
          longitudinal < -goal_xy_tolerance &&
          std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) - yaw)) <= .0872664626;
        const bool rear_curve = reverse_path && std::abs(rear_bearing) > .035;
        const double heading = distance <= goal_xy_tolerance ? yaw :
          (rear_curve ? yaw : wrapAngle(std::atan2(dy, dx) + (reverse_path ? kPi : 0.0)));
        const double bearing = wrapAngle(heading - yaw);
        // The shortest XY route is a straight line. Its initial and terminal
        // turns are explicit zero-translation cusps and are checked with the
        // whole body below. Keep a tangent arc as a separate candidate when
        // a safe turn in place is unavailable or its motion is preferable.
        const double radius = std::abs(bearing) > 1e-6 ?
          distance / (2.0 * std::sin(std::abs(bearing))) :
          std::numeric_limits<double>::infinity();
        const double arrival_yaw = yaw + 2.0 * bearing;
        const double terminal_error = std::abs(wrapAngle(
          tf2::getYaw(goal.pose.orientation) - arrival_yaw));
        const bool arc = direct_arc && std::abs(bearing) > 0.035 &&
          std::abs(bearing) <= 0.80 &&
          radius >= 0.90 && terminal_error <= 1.0;
        if (!std::isfinite(distance) || distance > 6.0 || (direct_arc && !arc))
        {
          return BT::NodeStatus::FAILURE;
        }
        path.header = global_->header;
        auto first = departure_pose;
        first.header = path.header;
        path.poses.push_back(first);
        if (direct && distance > goal_xy_tolerance && std::abs(bearing) > 0.035) {
          auto aligned = first;
          aligned.pose.orientation.z = std::sin(heading / 2);
          aligned.pose.orientation.w = std::cos(heading / 2);
          path.poses.push_back(aligned);
        }
        // Dense samples preserve the intended tangent through path pruning
        // and let the existing validator check the entire body on the arc.
        const double sweep = arc ? 2.0 * bearing : 0.0;
        const double length = arc ? radius * std::abs(sweep) : distance;
        const int samples = std::max(rear_curve ? 80 : 1,
          static_cast<int>(std::ceil(length / (rear_curve ? .01 : .025))));
        for (int i = 1; i < samples; ++i) {
          auto point = first;
          const double t = static_cast<double>(i) / samples;
          double tangent = heading;
          if (rear_curve) {
            // Cubic rear path: both endpoint tangents preserve chassis yaw.
            // Dense body poses are validated exactly like a straight route.
            const double goal_yaw = tf2::getYaw(goal.pose.orientation);
            const double handle = distance / 3.0;
            const double x0 = start.position.x, y0 = start.position.y;
            const double x1 = x0 - handle * std::cos(yaw);
            const double y1 = y0 - handle * std::sin(yaw);
            const double x3 = goal.pose.position.x, y3 = goal.pose.position.y;
            const double x2 = x3 + handle * std::cos(goal_yaw);
            const double y2 = y3 + handle * std::sin(goal_yaw);
            const double u = 1.0 - t;
            point.pose.position.x = u*u*u*x0 + 3*u*u*t*x1 + 3*u*t*t*x2 + t*t*t*x3;
            point.pose.position.y = u*u*u*y0 + 3*u*u*t*y1 + 3*u*t*t*y2 + t*t*t*y3;
            const double vx = u*u*(x1-x0) + 2*u*t*(x2-x1) + t*t*(x3-x2);
            const double vy = u*u*(y1-y0) + 2*u*t*(y2-y1) + t*t*(y3-y2);
            tangent = wrapAngle(std::atan2(vy, vx) + kPi);
          } else if (arc) {
            const double a = sweep * t;
            const double signed_radius = std::copysign(radius, bearing);
            point.pose.position.x += signed_radius *
              (std::sin(yaw + a) - std::sin(yaw));
            point.pose.position.y += signed_radius *
              (std::cos(yaw) - std::cos(yaw + a));
            tangent = yaw + a;
          } else {
            point.pose.position.x += t * dx;
            point.pose.position.y += t * dy;
          }
          point.pose.orientation.x = point.pose.orientation.y = 0.0;
          point.pose.orientation.z = std::sin(tangent / 2);
          point.pose.orientation.w = std::cos(tangent / 2);
          path.poses.push_back(point);
        }
        path.poses.push_back(goal);
      }
      if (path_query) {
        const auto planner_id = geometric_direct ?
          std::string(direct ? (reverse_path ? "DirectReverse" : "Direct") : "DirectArc") :
          getInput<std::string>("planner_id").value();
        if (reverse_path) {
          local_range = std::max(local_range, 1.05);
          std::vector<TrackingPose> poses;
          for (const auto & p : path.poses) {
            poses.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
          }
          if (!shortReversePath(poses)) {return BT::NodeStatus::FAILURE;}
        }
        const bool grid_planner = planner_id == "GridBased" || planner_id == "GridShortest";
        if (grid_planner && !getInput<bool>("prepared").value()) {
          // Smac2D short paths can contain default/identity quaternions.
          // XY-only planning has no in-place orientation primitives: derive
          // travel headings from geometry, retaining the requested goal yaw.
          // Installed Jazzy Smac2D emits integer-cell corners. Use physical
          // cell centres, avoiding fake initial turns on short XY routes.
          const double half_cell = global_->metadata.resolution * 0.5;
          for (auto & p : path.poses) {
            p.pose.position.x += half_cell;
            p.pose.position.y += half_cell;
          }
          geometry_msgs::msg::PoseStamped goal;
          if (approach_route) {(void)getInput("candidate_start", goal);}
          if ((approach_route || getInput("goal", goal)) && goal.header.frame_id == path.header.frame_id) {
            // Acceptance must be measured against the requested goal, not
            // Smac2D's discretized map-cell corner (up to a cell away).
            // The complete validator below still rejects an unsafe endpoint.
            path.poses.back().pose = goal.pose;
          }
          for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
            const auto & p = path.poses[i].pose.position;
            for (size_t j = i + 1; j < path.poses.size(); ++j) {
              const auto & next = path.poses[j].pose.position;
              if (std::hypot(next.x - p.x, next.y - p.y) < 1e-6) {continue;}
              const double a = std::atan2(next.y - p.y, next.x - p.x);
              auto & q = path.poses[i].pose.orientation;
              q.x = q.y = 0.0; q.z = std::sin(a / 2); q.w = std::cos(a / 2);
              break;
            }
          }
        }
        if (!geometric_direct && !grid_planner &&
          !getInput<bool>("prepared").value()) {
          geometry_msgs::msg::PoseStamped goal;
          if (approach_route) {(void)getInput("candidate_start", goal);}
          if ((approach_route || getInput("goal", goal)) && goal.header.frame_id == path.header.frame_id) {
            const auto & end = path.poses.back().pose.position;
            if (std::hypot(end.x - goal.pose.position.x, end.y - goal.pose.position.y) > 0.075) {
              return BT::NodeStatus::FAILURE;
            }
            // Lattice/hybrid endpoints are quantized too. Validate and follow
            // the requested XY/yaw, including its final short rotation.
            path.poses.back().pose = goal.pose;
          }
        }
        if (forward_route) {
          geometry_msgs::msg::PoseStamped candidate;
          geometry_msgs::msg::PoseStamped goal;
          if (!getInput("candidate_start", candidate) ||
            !getInput("goal", goal) || goal.header.frame_id != path.header.frame_id ||
            candidate.header.frame_id != path.header.frame_id ||
            std::hypot(path.poses.back().pose.position.x - goal.pose.position.x,
            path.poses.back().pose.position.y - goal.pose.position.y) > 0.075)
          {return BT::NodeStatus::FAILURE;}
          path.poses.back().pose = goal.pose;
          const auto & actual = map_pose.pose;
          const double yaw = tf2::getYaw(actual.orientation);
          const double dx = candidate.pose.position.x - actual.position.x;
          const double dy = candidate.pose.position.y - actual.position.y;
          const double advance = dx * std::cos(yaw) + dy * std::sin(yaw);
          const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
          if (!std::isfinite(advance) || advance < 0.10 || advance > 1.25 ||
            std::abs(lateral) > 0.02 ||
            std::abs(wrapAngle(tf2::getYaw(candidate.pose.orientation) - yaw)) > 0.05 ||
            std::hypot(path.poses.front().pose.position.x - candidate.pose.position.x,
            path.poses.front().pose.position.y - candidate.pose.position.y) > 0.075)
          {return BT::NodeStatus::FAILURE;}
          // Prepend the WHOLE actual forward departure, not just the route
          // starting at a hypothetical pocket. Keep lattice turn primitives.
          nav_msgs::msg::Path joined;
          joined.header = path.header;
          const int n = std::max(1, static_cast<int>(std::ceil(advance / 0.025)));
          for (int i = 0; i <= n; ++i) {
            auto p = map_pose;
            p.header = path.header;
            p.pose.position.x += dx * static_cast<double>(i) / n;
            p.pose.position.y += dy * static_cast<double>(i) / n;
            joined.poses.push_back(p);
          }
          // A quantized planner heading may need alignment AT the pocket,
          // never an unchecked coupled turn before reaching that location.
          auto aligned = candidate;
          aligned.header = path.header;
          aligned.pose.orientation = path.poses.front().pose.orientation;
          joined.poses.push_back(aligned);
          joined.poses.insert(joined.poses.end(), path.poses.begin(), path.poses.end());
          path = std::move(joined);
          local_range = std::max(1.0, advance + 0.05);
        }
        if (approach_route) {
          geometry_msgs::msg::PoseStamped approach, goal;
          if (!getInput("candidate_start", approach) || !getInput("goal", goal) ||
            approach.header.frame_id != path.header.frame_id ||
            goal.header.frame_id != path.header.frame_id || path.poses.size() < 2)
          {return BT::NodeStatus::FAILURE;}
          const double d = std::hypot(goal.pose.position.x - approach.pose.position.x,
            goal.pose.position.y - approach.pose.position.y);
          const double a = tf2::getYaw(goal.pose.orientation);
          if (d < .24 || d > .76 ||
            std::abs(wrapAngle(tf2::getYaw(approach.pose.orientation) - a)) > .01 ||
            std::hypot(approach.pose.position.x + d * std::cos(a) - goal.pose.position.x,
            approach.pose.position.y + d * std::sin(a) - goal.pose.position.y) > .01)
          {return BT::NodeStatus::FAILURE;}
          // Reach a clear standoff, align HERE, then advance to the exact goal.
          // This explicit cusp is also enforced by the controller's tracker.
          auto entry = path.poses.back();
          entry.pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
          path.poses.insert(path.poses.end() - 1, entry);
          const int n = static_cast<int>(std::ceil(d / .025));
          for (int i = 1; i <= n; ++i) {
            auto p = goal;
            const double remaining = d * (1.0 - static_cast<double>(i) / n);
            p.pose.position.x -= remaining * std::cos(a);
            p.pose.position.y -= remaining * std::sin(a);
            path.poses.push_back(p);
          }
        }
        // ControllerServer transforms end_pose_ using its stamp on every
        // goal check. The request's old stamp would freeze map->odom at launch
        // while RPP tracks with current localization. Use latest localization
        // for goal acceptance, consistently with path validation and tracking.
        path.poses.back().header.stamp = builtin_interfaces::msg::Time{};
        setOutput("checked_path", path);
        setOutput("path_planner", planner_id);
      }
      auto global_map = makeMap(*global_);
      auto local_map = makeMap(*local_);
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> gc(
        global_map.get()), lc(local_map.get());
      const auto to_local = tf_->lookupTransform(
        local_->header.frame_id, global_->header.frame_id, tf2::TimePointZero);
      const double tx = to_local.transform.translation.x;
      const double ty = to_local.transform.translation.y;
      const double ta = tf2::getYaw(to_local.transform.rotation);
      const double x = map_pose.pose.position.x;
      const double y = map_pose.pose.position.y;
      const double yaw = tf2::getYaw(map_pose.pose.orientation);
      auto collision = [&](double px, double py, double a) {
          if (occupied(gc, px, py, a, body)) {return true;}
          const double lx = tx + std::cos(ta) * px - std::sin(ta) * py;
          const double ly = ty + std::sin(ta) * px + std::cos(ta) * py;
          unsigned int mx, my;
          // Local observations are required for the launch sweep. Farther
          // along the route the global map checks the entire footprint.
          if (std::hypot(px - x, py - y) < local_range) {
            if (!local_map->worldToMap(lx, ly, mx, my)) {return true;}
            return occupied(lc, lx, ly, a + ta, body);
          }
          return false;
        };
      auto stable_rotation = [&](double angle, auto blocked) -> std::optional<double> {
          // A near-goal pivot must still fit after a small skid or AMCL shift.
          // Check both complete directions; prefer the short one only when
          // its swept footprint has the same reserve as the other direction.
          return chooseRotation(angle, [&](double signed_sweep) {
              const double stop_sweep = signed_sweep +
                std::copysign(0.06, signed_sweep);
              for (const auto & shift : std::array<std::pair<double, double>, 5>{{
                  {0.0, 0.0}, {0.04, 0.0}, {-0.04, 0.0},
                  {0.0, 0.04}, {0.0, -0.04}}})
              {
                if (!rotationSweepClear(yaw, stop_sweep, [&](double a) {
                    return blocked(x + shift.first, y + shift.second, a);
                  })) {return false;}
              }
              return true;
            });
        };
      auto checked_rotation = [&](double angle, auto blocked,
          bool robust) -> std::optional<double> {
          if (robust) {return stable_rotation(angle, blocked);}
          return chooseRotation(angle, [&](double signed_sweep) {
              return rotationSweepClear(yaw, signed_sweep, [&](double a) {
                  return blocked(x, y, a);
                });
            });
        };
      if (path_query && !geometric_direct && !getInput<bool>("prepared").value() &&
        !path.poses.empty())
      {
        const auto & end = path.poses.back().pose;
        const double distance = std::hypot(end.position.x - x, end.position.y - y);
        const double yaw_error = wrapAngle(tf2::getYaw(end.orientation) - yaw);
        if (distance <= goal_xy_tolerance && std::abs(yaw_error) > goal_yaw_tolerance) {
          const bool pivot_clear = chooseRotation(yaw_error, [&](double angle) {
              return rotationSweepClear(yaw, angle,
                [&](double a) {return collision(x, y, a);});
            }).has_value();
          // A long forward detour from an already reached XY can wedge the
          // rear against a wall. Use the fully checked rear corridor first
          // only when neither complete turn fits at the present pose.
          const bool rear_clear = !pivot_clear && poseSegmentClear(x, y, yaw,
              x - 0.16 * std::cos(yaw), y - 0.16 * std::sin(yaw), yaw,
              std::min(global_map->getResolution(), local_map->getResolution()),
              collision);
          // Keep a terminal escape already requested by the near-goal sweep.
          // Ranked route probes must not erase it before the recovery branch
          // gets to inspect the checked rear corridor.
          data_->terminal_escape_needed = data_->terminal_escape_needed ||
            (!pivot_clear && rear_clear);
          if (data_->terminal_escape_needed) {
            setOutput("blocked_near_robot", true);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Terminal XY reached but both turn sweeps blocked; use checked retreat before forward detour");
            return BT::NodeStatus::FAILURE;
          }
        }
      }
      auto terminal_alignment_clear = [&](double goal_yaw,
          bool robust = false) -> std::optional<double> {
          const double sweep = wrapAngle(goal_yaw - yaw);
          if (auto chosen = checked_rotation(sweep, collision, robust)) {return chosen;}
          // A final pivot may move AWAY from an existing map-only raster
          // contact in the extra padding or under the front fork tip. Clear
          // only such cells in a private validation map. The full footprint
          // must leave them promptly, never re-enter, and end clear on the
          // original map; live local obstacles block every sweep sample.
          auto core = body;
          nav2_costmap_2d::padFootprint(core, -0.03);
          const double lx = tx + std::cos(ta) * x - std::sin(ta) * y;
          const double ly = ty + std::sin(ta) * x + std::cos(ta) * y;
          const bool map_start_contact = occupied(gc, x, y, yaw, body);
          const bool local_start_contact = occupied(lc, lx, ly, yaw + ta, body);
          const bool goal_contact = collision(x, y, goal_yaw);
          if (!map_start_contact || local_start_contact || goal_contact) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Map-contact pivot unavailable: map_start=%d local_start=%d goal_contact=%d",
              map_start_contact, local_start_contact, goal_contact);
            return std::nullopt;
          }
          auto release_map = *global_map;
          nav2_costmap_2d::Costmap2D outer_mask(
            release_map.getSizeInCellsX(), release_map.getSizeInCellsY(),
            release_map.getResolution(), release_map.getOriginX(), release_map.getOriginY());
          auto inner_mask = outer_mask;
          auto initial_tip_sweep = outer_mask;
          nav2_costmap_2d::Footprint polygon;
          nav2_costmap_2d::transformFootprint(x, y, yaw, body, polygon);
          outer_mask.setConvexPolygonCost(polygon, 1);
          for (int direction : {-1, 1}) {
            for (int step = 0; step <= 25; ++step) {
              nav2_costmap_2d::transformFootprint(
                x, y, yaw + direction * step * 0.01, body, polygon);
              initial_tip_sweep.setConvexPolygonCost(polygon, 1);
            }
          }
          nav2_costmap_2d::transformFootprint(x, y, yaw, core, polygon);
          inner_mask.setConvexPolygonCost(polygon, 1);
          bool released = false;
          for (unsigned int iy = 0; iy < release_map.getSizeInCellsY(); ++iy) {
            for (unsigned int ix = 0; ix < release_map.getSizeInCellsX(); ++ix) {
              if (initial_tip_sweep.getCost(ix, iy) != 1 ||
                release_map.getCost(ix, iy) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
              double wx, wy;
              release_map.mapToWorld(ix, iy, wx, wy);
              const double dx = wx - x, dy = wy - y;
              const double forward = dx * std::cos(yaw) + dy * std::sin(yaw);
              const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
              const bool extra_padding = outer_mask.getCost(ix, iy) == 1 &&
                inner_mask.getCost(ix, iy) != 1;
              const bool fork_tip = initial_tip_sweep.getCost(ix, iy) == 1 &&
                forward >= 0.70 && forward <= 0.86 && std::abs(lateral) <= 0.16;
              if (!extra_padding && !fork_tip) {continue;}
              unsigned int mix, miy;
              if (!local_map->worldToMap(
                  tx + std::cos(ta) * wx - std::sin(ta) * wy,
                  ty + std::sin(ta) * wx + std::cos(ta) * wy, mix, miy) ||
                local_map->getCost(mix, miy) >= nav2_costmap_2d::LETHAL_OBSTACLE)
              {continue;}
              release_map.setCost(ix, iy, nav2_costmap_2d::FREE_SPACE);
              released = true;
            }
          }
          if (!released) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Map-contact pivot unavailable: no existing padding/fork-tip cell can be released");
            return std::nullopt;
          }
          nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> rc(
            &release_map);
          auto released_collision = [&](double px, double py, double a) {
              const bool original_contact = occupied(gc, px, py, a, body);
              const bool old_contact_returned = original_contact &&
                std::abs(a - yaw) > 0.25;
              const bool map_block = occupied(rc, px, py, a, body);
              const double ox = tx + std::cos(ta) * px - std::sin(ta) * py;
              const double oy = ty + std::sin(ta) * px + std::cos(ta) * py;
              const bool local_block = occupied(lc, ox, oy, a + ta, body);
              return old_contact_returned || map_block || local_block;
            };
          auto chosen = checked_rotation(sweep, released_collision, robust);
          if (chosen) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Terminal pivot releases existing map-only padding/fork-tip contact; "
              "live footprint and original-map final pose checked, remaining yaw %.1f deg",
              sweep * 180.0 / kPi);
          } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Map-contact pivot unavailable: remaining full-body sweep intersects an obstacle");
          }
          return chosen;
        };
      if (motion == "terminal_turn") {
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id ||
          std::hypot(goal.pose.position.x - x, goal.pose.position.y - y) >
          goal_xy_tolerance ||
          std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) - yaw)) <=
          goal_yaw_tolerance)
        {return BT::NodeStatus::FAILURE;}
        if (data_->turn_collision &&
          std::hypot(goal.pose.position.x - data_->collision_goal.pose.position.x,
          goal.pose.position.y - data_->collision_goal.pose.position.y) < 1e-6 &&
          std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
          tf2::getYaw(data_->collision_goal.pose.orientation))) < 1e-6)
        {
          data_->terminal_escape_needed = true;
          return BT::NodeStatus::FAILURE;
        }
        auto chosen = terminal_alignment_clear(tf2::getYaw(goal.pose.orientation), true);
        data_->terminal_escape_needed = !chosen;
        if (!chosen) {return BT::NodeStatus::FAILURE;}
        setOutput("turn_angle", *chosen);
        RCLCPP_INFO(node_->get_logger(),
          "Near-goal complete footprint sweep selected %.1f degrees",
          *chosen * 180.0 / kPi);
        return BT::NodeStatus::SUCCESS;
      }
      if (path_query && data_->terminal_escape_needed) {
        geometry_msgs::msg::PoseStamped goal;
        if (getInput("goal", goal) && goal.header.frame_id == global_->header.frame_id &&
          std::hypot(goal.pose.position.x - x, goal.pose.position.y - y) <=
          goal_xy_tolerance &&
          std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) - yaw)) >
          goal_yaw_tolerance)
        {
          setOutput("blocked_near_robot", true);
          return BT::NodeStatus::FAILURE;
        }
      }
      if (motion == "path" && !getInput<bool>("prepared").value() &&
        getInput<std::string>("planner_id").value() == "SE2Fallback")
      {
        const size_t rounded = roundCheckedLatticeCorners(path,
          std::min(global_map->getResolution(), local_map->getResolution()), collision);
        if (rounded > 0) {
          setOutput("checked_path", path);
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "Rounded %zu checked lattice corners into continuous turns", rounded);
        }
      }
      if (path_query && !getInput<bool>("prepared").value() && path.poses.size() >= 2) {
        // SimpleGoalChecker accepts XY as soon as the robot enters its disk.
        // Check the final yaw sweep THERE, not only at the exact goal
        // cell; a tangent arrival can fit at the endpoint yet become trapped
        // at the first accepted XY.
        const auto & end = path.poses.back().pose;
        const double gx = end.position.x, gy = end.position.y;
        const double goal_yaw = tf2::getYaw(end.orientation);
        // Do not reject a short XY route merely because its travel heading
        // differs from goal yaw. The full-body rotation sweep at the first
        // accepted XY below determines whether a terminal pivot is safe.
        bool outside = std::hypot(path.poses.front().pose.position.x - gx,
            path.poses.front().pose.position.y - gy) > goal_xy_tolerance + .005;
        for (size_t i = 1; i < path.poses.size(); ++i) {
          const auto & a = path.poses[i - 1].pose;
          const auto & b = path.poses[i].pose;
          const double dx = b.position.x - a.position.x;
          const double dy = b.position.y - a.position.y;
          const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / .01)));
          bool entered = false;
          for (int j = 1; j <= n; ++j) {
            const double t = static_cast<double>(j) / n;
            const double px = a.position.x + t * dx;
            const double py = a.position.y + t * dy;
            const double radius = std::hypot(px - gx, py - gy);
            if (radius > goal_xy_tolerance + .005) {outside = true;}
            if (!outside || radius > goal_xy_tolerance - .002) {continue;}
            const double approach_yaw = tf2::getYaw(a.orientation);
            const double sweep = wrapAngle(goal_yaw - approach_yaw);
            auto alignment_clear = [&](double cx, double cy) {
                const double remaining = std::hypot(gx - cx, gy - cy);
                const double target = remaining > goal_xy_tolerance ?
                  wrapAngle(std::atan2(gy - cy, gx - cx) + (reverse_path ? kPi : 0.0)) : goal_yaw;
                return static_cast<bool>(chooseRotation(
                    wrapAngle(target - approach_yaw),
                    [&](double angle) {
                      return rotationSweepClear(approach_yaw, angle,
                        [&](double heading) {return collision(cx, cy, heading);});
                    }, remaining <= goal_xy_tolerance));
              };
            bool clear = std::abs(sweep) <= goal_yaw_tolerance ?
              !collision(px, py, approach_yaw) : alignment_clear(px, py);
            if (clear && std::abs(sweep) > .15) {
              // Allow for a few centimetres of path-tracking error before
              // entering the XY tolerance. This only tightens
              // candidates that still require a substantial final turn.
              const double nx = -std::sin(approach_yaw);
              const double ny = std::cos(approach_yaw);
              for (double back : {0.0, .04}) {
                for (double lateral : {-.04, .04}) {
                  clear = clear && alignment_clear(
                    px - back * (reverse_path ? -1 : 1) * std::cos(approach_yaw) + lateral * nx,
                    py - back * (reverse_path ? -1 : 1) * std::sin(approach_yaw) + lateral * ny);
                }
              }
            }
            if (!clear && direct && !reverse_path &&
              poseSegmentClear(px, py, approach_yaw, gx, gy, approach_yaw,
                std::min(global_map->getResolution(), local_map->getResolution()), collision) &&
              alignment_clear(gx, gy))
            {
              // The robot can stay on this straight line until the exact
              // endpoint, then make the checked final yaw correction there.
              // FollowPath must use the same deferred-pivot rule at runtime.
              clear = true;
            }
            if (!clear)
            {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                "First accepted XY cannot align goal yaw at (%.3f, %.3f)", px, py);
              setOutput("blocked_near_robot", std::hypot(px - x, py - y) < 1.0);
              return BT::NodeStatus::FAILURE;
            }
            entered = true;
            break;
          }
          if (entered) {break;}
        }
      }
      if (motion == "goal_approach_candidate") {
        geometry_msgs::msg::PoseStamped goal;
        const double d = getInput<double>("candidate_distance").value();
        if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id ||
          !std::isfinite(d) || d < .25 || d > .75)
        {return BT::NodeStatus::FAILURE;}
        auto approach = goal;
        const double a = tf2::getYaw(goal.pose.orientation);
        approach.pose.position.x -= d * std::cos(a);
        approach.pose.position.y -= d * std::sin(a);
        if (!poseSegmentClear(approach.pose.position.x, approach.pose.position.y, a,
            goal.pose.position.x, goal.pose.position.y, a,
            global_map->getResolution(), collision))
        {return BT::NodeStatus::FAILURE;}
        setOutput("approach_goal", approach);
        return BT::NodeStatus::SUCCESS;
      }
      if (!path_query) {
        return checkManeuver(motion, x, y, yaw, ta, tx, ty, body, gc, lc);
      }
      setOutput("blocked_near_robot", false);
      // Trim the traversed prefix when revalidating a stable plan.
      size_t nearest = 0;
      double distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose.position;
        const double d = std::hypot(p.x - x, p.y - y);
        if (d < distance) {distance = d; nearest = i;}
      }
      double remaining_length = 0.0;
      for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
        const auto & a = path.poses[i - 1].pose.position;
        const auto & b = path.poses[i].pose.position;
        remaining_length += std::hypot(b.x - a.x, b.y - a.y);
      }
      if (getInput<bool>("prepared").value() || direct) {
        const auto & goal = path.poses.back().pose;
        const bool short_pivot_plan = getInput<bool>("prepared").value() &&
          getInput<std::string>("planner_id").value() == "Direct" && path.poses.size() == 2 &&
          std::hypot(path.poses.front().pose.position.x - goal.position.x,
          path.poses.front().pose.position.y - goal.position.y) <= goal_xy_tolerance;
        if (short_pivot_plan &&
          std::hypot(path.poses.front().pose.position.x - x,
          path.poses.front().pose.position.y - y) <= 0.12 &&
          std::hypot(goal.position.x - x, goal.position.y - y) <= 0.12 &&
          std::abs(wrapAngle(tf2::getYaw(goal.orientation) - yaw)) > goal_yaw_tolerance)
        {
          // This path was selected as a turn at an already accepted XY.
          // During a skid-steer pivot, a small pose shift can make the old
          // launch point nearer than the goal. Rechecking its original yaw
          // then asks the robot to reverse the completed part of the turn.
          // Validate only the remaining turn from the ACTUAL pose, with the
          // current global and live local footprints. The goal checker still
          // enforces the original 5 cm / 5 degree arrival tolerance.
          const bool clear = static_cast<bool>(
            terminal_alignment_clear(tf2::getYaw(goal.orientation)));
          setOutput("blocked_near_robot", !clear);
          return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        if (remaining_length <= 0.15 &&
          std::hypot(goal.position.x - x, goal.position.y - y) <= goal_xy_tolerance)
        {
          const double goal_yaw = tf2::getYaw(goal.orientation);
          bool clear = static_cast<bool>(terminal_alignment_clear(goal_yaw));
          if (!clear && !reverse_path) {
            const double dx = goal.position.x - x, dy = goal.position.y - y;
            const double forward = dx * std::cos(yaw) + dy * std::sin(yaw);
            const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
            const double lx = tx + std::cos(ta) * x - std::sin(ta) * y;
            const double ly = ty + std::sin(ta) * x + std::cos(ta) * y;
            const bool local_turn_blocked = !chooseRotation(
              wrapAngle(goal_yaw - yaw), [&](double angle) {
                return rotationSweepClear(yaw, angle, [&](double a) {
                    return occupied(lc, lx, ly, a + ta, body);
                  });
              });
            if (forward > 0.015 && std::abs(lateral) < 0.9 * goal_xy_tolerance &&
              local_turn_blocked &&
              poseSegmentClear(x, y, yaw, goal.position.x, goal.position.y, yaw,
                std::min(global_map->getResolution(), local_map->getResolution()), collision) &&
              chooseRotation(wrapAngle(goal_yaw - yaw), [&](double angle) {
                return rotationSweepClear(yaw, angle,
                  [&](double a) {return collision(goal.position.x, goal.position.y, a);});
              }))
            {
              clear = true;
            }
          }
          if (!clear && std::abs(wrapAngle(goal_yaw - yaw)) > goal_yaw_tolerance &&
            !chooseRotation(wrapAngle(goal_yaw - yaw), [&](double angle) {
              return rotationSweepClear(yaw, angle,
                [&](double a) {return collision(x, y, a);});
            }) &&
            poseSegmentClear(x, y, yaw,
              x - 0.16 * std::cos(yaw), y - 0.16 * std::sin(yaw), yaw,
              std::min(global_map->getResolution(), local_map->getResolution()),
              collision))
          {
            data_->terminal_escape_needed = true;
          } else if (clear) {
            data_->terminal_escape_needed = false;
          }
          setOutput("blocked_near_robot", !clear);
          return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
      }
      if (getInput<bool>("prepared").value() && distance <= 0.015) {
        // At a stationary-turn cusp, only the REMAINING sweep is relevant.
        // A nearest-XY tie otherwise chooses the pre-turn heading and asks
        // validation to sweep back through already-completed orientations.
        // Match the controller's 1.5 cm turn-entry threshold; farther away,
        // preserve the forward departure before any rotation is permitted.
        while (nearest + 1 < path.poses.size() &&
          std::hypot(path.poses[nearest + 1].pose.position.x -
          path.poses[nearest].pose.position.x, path.poses[nearest + 1].pose.position.y -
          path.poses[nearest].pose.position.y) < 0.005)
        {
          ++nearest;
        }
      }
      const auto & first = path.poses[nearest].pose;
      const double first_yaw = tf2::getYaw(first.orientation);
      double launch_yaw = first_yaw;
      if (getInput<bool>("prepared").value() && nearest + 1 == path.poses.size()) {
        // The closest point can be the goal while still outside XY tolerance.
        // FollowPath approaches it first; the final goal yaw must not become
        // a premature in-place rotation at the current, offset position.
        const double remaining = std::hypot(first.position.x - x, first.position.y - y);
        if (remaining > goal_xy_tolerance) {
          launch_yaw = wrapAngle(std::atan2(first.position.y - y, first.position.x - x) +
            (reverse_path ? kPi : 0.0));
        }
      }
      bool starts_forward = false;
      for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose.position;
        if (std::hypot(p.x - x, p.y - y) >= 0.05) {
          starts_forward = std::abs(wrapAngle(std::atan2(p.y - y, p.x - x) - yaw)) < 0.35;
          break;
        }
        // A rotation primitive at the launch point requires an actual sweep.
        if (std::abs(wrapAngle(tf2::getYaw(path.poses[i].pose.orientation) - first_yaw)) >
          0.02)
        {
          break;
        }
      }
      if (getInput<bool>("prepared").value() && nearest + 1 == path.poses.size()) {
        const auto & p = path.poses.back().pose.position;
        const double dx = p.x - x, dy = p.y - y;
        // Match the controller's short terminal tracking phase. The full
        // translation and final sweep below must still fit before proceeding.
        starts_forward = canApproachGoalWithoutStationaryTurn(
          dx * std::cos(yaw) + dy * std::sin(yaw),
          -dx * std::sin(yaw) + dy * std::cos(yaw), remaining_length, goal_xy_tolerance);
      }
      if ((!geometric_direct || forward_direct) && starts_forward && !occupied(lc,
          tx + std::cos(ta) * x - std::sin(ta) * y,
          ty + std::sin(ta) * x + std::cos(ta) * y, yaw + ta, body))
      {
        // Allow departure from a raster overlap confined to the extra 3 cm
        // padding, only when the live local footprint is clear. Never erase
        // the core body, unknown cells, or any newly encountered obstacles.
        // These are private validation copies, not the actual costmaps.
        auto inner = body;
        nav2_costmap_2d::padFootprint(inner, -0.03);
        nav2_costmap_2d::Footprint outer_polygon, inner_polygon;
        nav2_costmap_2d::transformFootprint(x, y, yaw, body, outer_polygon);
        nav2_costmap_2d::transformFootprint(x, y, yaw, inner, inner_polygon);
        nav2_costmap_2d::Costmap2D outer_mask(
          global_map->getSizeInCellsX(), global_map->getSizeInCellsY(),
          global_map->getResolution(), global_map->getOriginX(), global_map->getOriginY());
        auto inner_mask = outer_mask;
        outer_mask.setConvexPolygonCost(outer_polygon, 1);
        inner_mask.setConvexPolygonCost(inner_polygon, 1);
        for (unsigned int iy = 0; iy < global_map->getSizeInCellsY(); ++iy) {
          for (unsigned int ix = 0; ix < global_map->getSizeInCellsX(); ++ix) {
            if (outer_mask.getCost(ix, iy) != 1 || inner_mask.getCost(ix, iy) == 1 ||
              global_map->getCost(ix, iy) != nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              continue;
            }
            double wx, wy;
            global_map->mapToWorld(ix, iy, wx, wy);
            const double lx = tx + std::cos(ta) * wx - std::sin(ta) * wy;
            const double ly = ty + std::sin(ta) * wx + std::cos(ta) * wy;
            unsigned int mx, my;
            if (local_map->worldToMap(lx, ly, mx, my) &&
              local_map->getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              global_map->setCost(ix, iy, nav2_costmap_2d::FREE_SPACE);
            }
          }
        }
      }
      if (!starts_forward && !chooseRotation(wrapAngle(launch_yaw - yaw), [&](double sweep) {
          return rotationSweepClear(yaw, sweep, [&](double a) {
                   return collision(x, y, a);
          });
        }, nearest + 1 < path.poses.size()))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "Initial footprint rotation blocked at (%.3f, %.3f), angle %.1f deg",
          x, y, wrapAngle(launch_yaw - yaw) * 180.0 / kPi);
        setOutput("blocked_near_robot", true);
        return BT::NodeStatus::FAILURE;
      }
      const double resolution = std::min(
        global_map->getResolution(), local_map->getResolution());
      // Heading quantization (e.g. 99 deg robot vs 90 deg lattice) does not
      // require a launch spin when the controller can align while advancing.
      // Check that coupled motion from the real current yaw instead.
      double px = x, py = y, pa = starts_forward ? yaw : launch_yaw;
      if (collision(x, y, yaw)) {
        setOutput("blocked_near_robot", true);
        return BT::NodeStatus::FAILURE;
      }
      const size_t begin = starts_forward && distance < 0.03 ? nearest + 1 : nearest;
      for (size_t i = begin; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose;
        const double a = tf2::getYaw(p.orientation);
        // RPP reaches the final XY first, then aligns the goal heading.
        // At arrival validate the complete chosen terminal yaw sweep.
        if (i + 1 == path.poses.size()) {
          if (!poseSegmentClear(
              px, py, pa, p.position.x, p.position.y, pa, resolution, collision) ||
            !chooseRotation(wrapAngle(a - pa), [&](double sweep) {
              return rotationSweepClear(pa, sweep, [&](double heading) {
                       return collision(p.position.x, p.position.y, heading);
              });
            }))
          {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Terminal footprint / short yaw correction blocked at (%.3f, %.3f)",
              p.position.x, p.position.y);
            setOutput("blocked_near_robot", std::hypot(p.position.x - x, p.position.y - y) < 1.0);
            return BT::NodeStatus::FAILURE;
          }
          break;
        }
        if (!poseSegmentClear(
            px, py, pa, p.position.x, p.position.y, a, resolution, collision))
        {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "Path footprint blocked at pose %zu (%.3f, %.3f), yaw %.1f deg", i,
            p.position.x, p.position.y, a * 180.0 / kPi);
          setOutput("blocked_near_robot", std::hypot(p.position.x - x, p.position.y - y) < 1.0);
          return BT::NodeStatus::FAILURE;
        }
        px = p.position.x; py = p.position.y; pa = a;
      }
      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "Footprint route check unavailable: %s", error.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  BT::NodeStatus checkManeuver(
    const std::string & mode, double x, double y, double yaw, double ta,
    double tx, double ty, const nav2_costmap_2d::Footprint & body,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & gc,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & lc)
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id) {
      return BT::NodeStatus::FAILURE;
    }
    const auto same_goal = [&](const geometry_msgs::msg::PoseStamped & previous) {
        return previous.header.frame_id == goal.header.frame_id &&
          std::hypot(goal.pose.position.x - previous.pose.position.x,
          goal.pose.position.y - previous.pose.position.y) < 1e-6 &&
          std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
          tf2::getYaw(previous.pose.orientation))) < 1e-6;
      };
    if (!same_goal(data_->turn_goal)) {
      data_->near_goal_reverse_pending = data_->near_goal_reverse_done = false;
      data_->turn_attempts.clear();
      data_->escape_memory.reset();
      data_->turn_geometric_stop = data_->selected_turn_goal_facing = false;
      data_->turn_handoff = false;
      data_->turn_goal = goal;
      ++data_->motion_revision;
    }
    if (!same_goal(data_->forward_goal)) {
      data_->forward_active = data_->forward_geometric_stop = false;
      data_->forward_goal = goal;
      ++data_->motion_revision;
    }
    if (data_->turn_collision &&
      (std::hypot(goal.pose.position.x - data_->collision_goal.pose.position.x,
      goal.pose.position.y - data_->collision_goal.pose.position.y) > 1e-6 ||
      std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
      tf2::getYaw(data_->collision_goal.pose.orientation))) > 1e-6))
    {
      data_->turn_collision = false;
      data_->failed_forward = false;
      ++data_->motion_revision;
    }
    if ((data_->retreat_active || data_->retreat_stopped) &&
      (std::hypot(goal.pose.position.x - data_->retreat_goal.pose.position.x,
      goal.pose.position.y - data_->retreat_goal.pose.position.y) > 1e-6 ||
      std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
      tf2::getYaw(data_->retreat_goal.pose.orientation))) > 1e-6))
    {
      data_->retreat_active = data_->retreat_stopped = false;
      ++data_->motion_revision;
    }
    if (mode == "replan_after_retreat") {
      const auto error = getInput<uint16_t>("motion_error").value();
      const bool collision_stop = error == nav2_msgs::action::BackUp::Result::COLLISION_AHEAD;
      if ((!data_->retreat_stopped && !(data_->retreat_active && collision_stop)) ||
        (error != nav2_msgs::action::BackUp::Result::NONE && !collision_stop))
      {
        return BT::NodeStatus::FAILURE;
      }
      // Use odometry, not map localization: a pose correction or movement
      // toward the old dead end cannot authorize a new recovery cycle.
      const double ox = tx + std::cos(ta) * x - std::sin(ta) * y;
      const double oy = ty + std::sin(ta) * x + std::cos(ta) * y;
      const double retreat = -(ox - data_->retreat_x) * std::cos(data_->retreat_yaw) -
        (oy - data_->retreat_y) * std::sin(data_->retreat_yaw);
      data_->retreat_active = data_->retreat_stopped = false;
      ++data_->motion_revision;
      if (retreat < 0.05) {return BT::NodeStatus::FAILURE;}
      RCLCPP_INFO(node_->get_logger(),
        "Retreat stopped after %.2f m measured travel; replan from the new pose before abort",
        retreat);
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "recovery_allowed" || mode == "motion_fault_free") {
      const auto planner = getInput<uint16_t>("planner_error").value();
      const auto controller = getInput<uint16_t>("controller_error").value();
      const bool near = getInput<bool>("local_blocked").value();
      // A distant blocked destination is not evidence that backing up here
      // will help. Nor should failed terminal yaw alignment move the goal XY.
      const bool healthy = controller == 0 || controller == 104 ||
        controller == 105 || controller == 106;
      if (mode == "motion_fault_free") {
        // This gate permits complete checked route searches, not local escape.
        // A close goal may need alignment before returning to its final pose.
        return healthy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      const double dx = goal.pose.position.x - x;
      const double dy = goal.pose.position.y - y;
      const double goal_distance = std::hypot(dx, dy);
      const double goal_forward = dx * std::cos(yaw) + dy * std::sin(yaw);
      const double local_x = tx + std::cos(ta) * x - std::sin(ta) * y;
      const double local_y = ty + std::sin(ta) * x + std::cos(ta) * y;
      // A nearby goal behind the chassis may be unreachable to a forward
      // planner when a static-map cell already lies under the front fork.
      // Permit the recovery branch to inspect the full rear corridor; this
      // gate alone never commands motion or clears an obstacle.
      const bool mapped_start_contact = occupied(gc, x, y, yaw, body) &&
        !occupied(lc, local_x, local_y, yaw + ta, body);
      const bool rear_escape_candidate = mapped_start_contact &&
        goal_distance > .05 && goal_distance <= 1.0 && goal_forward < -.10 &&
        (planner == 205 || planner == 207 || planner == 208);
      // A very close goal behind the current heading can require a checked
      // rear departure before any forward-only planner finds a short route.
      // A bounded planner search timeout is not a sensor/TF fault. The rear
      // corridor still needs a fresh full-body check before any movement.
      const bool close_behind = goal_distance > .05 && goal_distance < .35 &&
        goal_forward < -.04 &&
        (planner == 0 || planner == 205 || planner == 207 || planner == 208);
      const bool local_escape_needed = near || data_->escape_memory.committed ||
        data_->turn_limited;
      const bool no_local_route = local_escape_needed &&
        planner == nav2_msgs::action::ComputePathToPose::Result::NO_VALID_PATH;
      const bool terminal_escape = data_->terminal_escape_needed &&
        goal_distance <= 0.05 &&
        std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) - yaw)) > .0872664626;
      const bool permitted = healthy && (goal_distance > 0.05 || terminal_escape) &&
        (terminal_escape || rear_escape_candidate || close_behind || planner == 205 ||
        no_local_route || (planner == 0 &&
        (local_escape_needed || controller == 104 || controller == 105 ||
        controller == 106 || data_->turn_limited)));
      if (!permitted && goal_distance > .05 && goal_distance < .35 &&
        dx * std::cos(yaw) + dy * std::sin(yaw) < -.04)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "Near-behind escape unavailable: planner=%u controller=%u local_blocked=%d",
          planner, controller, near);
      }
      if (permitted && (rear_escape_candidate || close_behind || terminal_escape) &&
        !data_->near_goal_reverse_done &&
        !data_->near_goal_reverse_pending)
      {
        data_->near_goal_reverse_pending = true;
        data_->near_goal_reverse_x = tx + std::cos(ta) * x - std::sin(ta) * y;
        data_->near_goal_reverse_y = ty + std::sin(ta) * x + std::cos(ta) * y;
        data_->near_goal_reverse_yaw = yaw + ta;
        ++data_->motion_revision;
        RCLCPP_INFO(node_->get_logger(),
          "Near goal with blocked turn: check rear travel before choosing a turn");
      }
      return permitted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    const double odom_x = tx + std::cos(ta) * x - std::sin(ta) * y;
    const double odom_y = ty + std::sin(ta) * x + std::cos(ta) * y;
    const bool translation_required = data_->turnTranslationRequired(odom_x, odom_y);
    if (data_->escape_memory.observe(odom_x, odom_y)) {
      ++data_->motion_revision;
      RCLCPP_INFO(node_->get_logger(), "Escape reached a different odometry region; reconsider exits");
    }
    // Reverse guards run at BT frequency, but expensive swept-body queries
    // need only run at 5 Hz. Freshness/TF checks still run on EVERY tick.
    const auto now = std::chrono::steady_clock::now();
    if ((mode == "reverse_needed" || mode == "escape_available" || mode == "forward_safe") &&
      cached_mode_ == mode && cached_revision_ == data_->motion_revision &&
      now - cached_time_ < std::chrono::milliseconds(200))
    {
      return cached_status_;
    }
    auto local_collision = [&](double px, double py, double a) {
        return occupied(lc, tx + std::cos(ta) * px - std::sin(ta) * py,
                 ty + std::sin(ta) * px + std::cos(ta) * py, a + ta, body);
      };
    const double resolution = std::min(gc.getCostmap()->getResolution(),
      lc.getCostmap()->getResolution());
    auto forward_at = [&](double px, double py, double a, double distance, auto blocked) {
        return poseSegmentClear(px, py, a, px + distance * std::cos(a),
                 py + distance * std::sin(a), a, resolution, blocked);
      };
    // Start-occupied escape is translation OUT of an existing map overlap,
    // never a general static-map clearing. The unmodified endpoint must fit.
    // Any newly intersected global cell and every live local obstacle remain
    // blocking. No published costmap or physical footprint is changed.
    auto departure_map = *gc.getCostmap();
    nav2_costmap_2d::Footprint polygon;
    nav2_costmap_2d::transformFootprint(x, y, yaw, body, polygon);
    nav2_costmap_2d::Costmap2D mask(departure_map.getSizeInCellsX(),
      departure_map.getSizeInCellsY(), departure_map.getResolution(),
      departure_map.getOriginX(), departure_map.getOriginY());
    mask.setConvexPolygonCost(polygon, 1);
    if (!local_collision(x, y, yaw)) {
      for (unsigned int iy = 0; iy < departure_map.getSizeInCellsY(); ++iy) {
        for (unsigned int ix = 0; ix < departure_map.getSizeInCellsX(); ++ix) {
          if (mask.getCost(ix, iy) == 1 && departure_map.getCost(ix, iy) == 254) {
            departure_map.setCost(ix, iy, 0);
          }
        }
      }
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> dc(&departure_map);
    auto departure_collision = [&](double px, double py, double a) {
        return occupied(dc, px, py, a, body) || local_collision(px, py, a);
      };
    // Recovery must check live observations over the ENTIRE manoeuvre,
    // including a turning pocket more than one metre from the start.
    auto maneuver_collision = [&](double px, double py, double a) {
        return occupied(gc, px, py, a, body) || local_collision(px, py, a);
      };
    // Allow a straight retreat out of an existing map-only front fork-tip or
    // fork-side contact, as well as an extra-padding contact. The cell must lie inside
    // the starting body, be clear in the live local map, and be left behind
    // by the checked reverse motion. Never release unknown or new obstacles.
    // Use a private map; the published costmap remains untouched.
    auto retreat_map = *gc.getCostmap();
    auto core = body;
    nav2_costmap_2d::padFootprint(core, -0.03);
    bool map_contact_release = false;
    if (occupied(gc, x, y, yaw, body) &&
      !local_collision(x, y, yaw))
    {
      nav2_costmap_2d::Footprint core_polygon;
      nav2_costmap_2d::transformFootprint(x, y, yaw, core, core_polygon);
      auto core_mask = mask;
      core_mask.resetMap(0, 0, core_mask.getSizeInCellsX(), core_mask.getSizeInCellsY());
      core_mask.setConvexPolygonCost(core_polygon, 1);
      for (unsigned int iy = 0; iy < retreat_map.getSizeInCellsY(); ++iy) {
        for (unsigned int ix = 0; ix < retreat_map.getSizeInCellsX(); ++ix) {
          if (mask.getCost(ix, iy) == 1 &&
            retreat_map.getCost(ix, iy) == nav2_costmap_2d::LETHAL_OBSTACLE)
          {
            double wx, wy;
            retreat_map.mapToWorld(ix, iy, wx, wy);
            const double dx = wx - x, dy = wy - y;
            const double forward = dx * std::cos(yaw) + dy * std::sin(yaw);
            const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
            const bool extra_padding = core_mask.getCost(ix, iy) != 1;
            const bool front_fork_side = forward >= 0.40 &&
              std::abs(lateral) >= 0.13;
            const bool front_fork_tip = forward >= 0.70 && forward <= 0.86 &&
              std::abs(lateral) <= 0.16;
            if (!extra_padding && !front_fork_side && !front_fork_tip) {continue;}
            unsigned int lx, ly;
            if (lc.getCostmap()->worldToMap(
                tx + std::cos(ta) * wx - std::sin(ta) * wy,
                ty + std::sin(ta) * wx + std::cos(ta) * wy, lx, ly) &&
              lc.getCostmap()->getCost(lx, ly) < nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              retreat_map.setCost(ix, iy, nav2_costmap_2d::FREE_SPACE);
              map_contact_release = true;
            }
          }
        }
      }
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> rc(&retreat_map);
    auto retreat_collision = [&](double px, double py, double a) {
        return occupied(rc, px, py, a, body) || local_collision(px, py, a);
      };
    auto rear_clear = [&]() {
        if (forward_at(x, y, yaw, -0.16, maneuver_collision)) {return true;}
        return map_contact_release && forward_at(x, y, yaw, -0.30, retreat_collision) &&
          !maneuver_collision(x - 0.30 * std::cos(yaw), y - 0.30 * std::sin(yaw), yaw);
      };
    if (mode == "turn_collision") {
      // A geometry stop can be followed by another checked retreat. Never
      // turn a motor timeout, TF failure or unknown fault into reverse motion.
      if (getInput<uint16_t>("motion_error").value() !=
        nav2_msgs::action::Spin::Result::COLLISION_AHEAD)
      {
        return BT::NodeStatus::FAILURE;
      }
      data_->turn_collision = true;
      if (std::hypot(goal.pose.position.x - x, goal.pose.position.y - y) <= .05) {
        data_->terminal_escape_needed = true;
      }
      data_->collision_min_retreat = 0.15;
      data_->failed_forward = false;
      data_->collision_goal = goal;
      data_->collision_x = tx + std::cos(ta) * x - std::sin(ta) * y;
      data_->collision_y = ty + std::sin(ta) * x + std::cos(ta) * y;
      data_->collision_yaw = yaw + ta;
      ++data_->motion_revision;
      const bool rear = rear_clear();
      // Log the heading-exclusion policy, not a blanket retreat-distance rule.
      RCLCPP_WARN(node_->get_logger(),
        "Spin collision: %s; exclude attempted heading, check other exits before retreat",
        rear ? "rear checked, continuing escape" : "rear blocked, stopping safely");
      return rear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    // Measure signed rear travel in odom: an AMCL correction or just turning
    // at the same position must not count as having backed farther out.
    const double retreat = -(odom_x - data_->collision_x) * std::cos(data_->collision_yaw) -
      (odom_y - data_->collision_y) * std::sin(data_->collision_yaw);
    const bool recorded_turn_here = std::any_of(
      data_->turn_attempts.begin(), data_->turn_attempts.end(), [&](const auto & attempt) {
        return std::hypot(odom_x - attempt.x, odom_y - attempt.y) < 0.15;
      });
    // The attempted heading is excluded by useful_exit. Another fully checked
    // turn can execute immediately; only an unrecorded collision result needs
    // the conservative measured retreat before selecting a turn again.
    if (data_->near_goal_reverse_pending) {
      const double rear_travel = -(
        (odom_x - data_->near_goal_reverse_x) * std::cos(data_->near_goal_reverse_yaw) +
        (odom_y - data_->near_goal_reverse_y) * std::sin(data_->near_goal_reverse_yaw));
      if (rear_travel >= .25) {
        data_->near_goal_reverse_pending = false;
        data_->near_goal_reverse_done = true;
        ++data_->motion_revision;
      }
    }
    const bool force_retreat = data_->near_goal_reverse_pending ||
      (data_->turn_collision && !recorded_turn_here &&
      retreat < data_->collision_min_retreat);
    if (mode == "forward_candidate") {
      const double d = getInput<double>("candidate_distance").value();
      const double dx = goal.pose.position.x - x;
      const double dy = goal.pose.position.y - y;
      const double goal_distance = std::hypot(dx, dy);
      const bool close_behind = goal_distance < .35 &&
        dx * std::cos(yaw) + dy * std::sin(yaw) < -.04;
      if (close_behind || force_retreat || data_->escape_memory.committed ||
        !std::isfinite(goal_distance) ||
        !std::isfinite(d) || d < 0.15 || d > 1.20 ||
        d > goal_distance + 0.04 ||
        !forward_at(x, y, yaw, d, maneuver_collision)) {return BT::NodeStatus::FAILURE;}
      geometry_msgs::msg::PoseStamped start;
      start.header = global_->header;
      start.pose.position.x = x + d * std::cos(yaw);
      start.pose.position.y = y + d * std::sin(yaw);
      start.pose.orientation.z = std::sin(yaw / 2);
      start.pose.orientation.w = std::cos(yaw / 2);
      setOutput("forward_start", start);
      return BT::NodeStatus::SUCCESS;
    }
    // Turning pockets need extra room for raster/localization error and the
    // measured braking overrun. The actual translation/rear footprint is
    // unchanged; this margin is only for deciding it is time to stop backing.
    auto turning_body = body;
    nav2_costmap_2d::padFootprint(turning_body, 0.04);
    auto pocket_collision = [&](double px, double py, double a) {
        return occupied(gc, px, py, a, turning_body) || occupied(lc,
                 tx + std::cos(ta) * px - std::sin(ta) * py,
                 ty + std::sin(ta) * px + std::cos(ta) * py, a + ta, turning_body);
      };
    auto turn_fits = [&](double px, double py, double from, double angle) {
        const double sweep = std::abs(angle) < 1e-6 ? angle :
          angle + std::copysign(0.07, angle);
        return rotationSweepClear(from, sweep, [&](double a) {
                   return pocket_collision(px, py, a);
        });
      };
    struct Exit {bool clear{false}; double turn{0.0};};
    auto useful_exit = [&](double px, double py, bool require_full_turn = false) -> Exit {
        if (maneuver_collision(px, py, yaw)) {return {};}
        const double dx = goal.pose.position.x - px;
        const double dy = goal.pose.position.y - py;
        const double distance = std::hypot(dx, dy);
        const double desired = wrapAngle(std::atan2(dy, dx) - yaw);
        // An open 15 cm ahead is NOT an exit if the same corridor ends at
        // a wall. Check a longer goal-directed corridor before stopping a
        // retreat. At an accepted XY only the terminal yaw is relevant.
        if (distance <= 0.05) {
          const double a = wrapAngle(tf2::getYaw(goal.pose.orientation) - yaw);
          const auto chosen = chooseRotation(a, [&](double angle) {
              return turn_fits(px, py, yaw, angle);
            });
          return chosen ? Exit{true, *chosen} : Exit{};
        }
        if (!(data_->failed_forward && retreat < 0.05) && std::abs(desired) <= 0.10 &&
          forward_at(px, py, yaw, std::min(0.75, distance), maneuver_collision))
        {
          return {true, 0.0};
        }
        std::vector<double> angles{desired};
        for (int i = 1; !data_->escape_memory.committed && !require_full_turn && i <= 12; ++i) {
          angles.push_back(i * kPi / 12);
          angles.push_back(-i * kPi / 12);
        }
        // Prefer a heading toward the goal, then the smaller signed sweep.
        // A tiny turn still pointing into the old dead end is not progress.
        std::sort(angles.begin(), angles.end(), [&](double a, double b) {
            return 2.0 * std::abs(wrapAngle(desired - a)) + std::abs(a) <
                   2.0 * std::abs(wrapAngle(desired - b)) + std::abs(b);
          });
        for (const double a : angles) {
          if (std::abs(a) < 0.10) {continue;}
          const double ox = tx + std::cos(ta) * px - std::sin(ta) * py;
          const double oy = ty + std::sin(ta) * px + std::cos(ta) * py;
          bool tried = false;
          for (const auto & attempt : data_->turn_attempts) {
            if (std::hypot(ox - attempt.x, oy - attempt.y) < 0.15 &&
              std::abs(wrapAngle(yaw + ta + a - attempt.heading)) < 0.12)
            {tried = true; break;}
          }
          if (tried) {continue;}
          const double exit_distance = std::min(0.75, distance);
          if (!turn_fits(px, py, yaw, a) ||
            !forward_at(px, py, yaw + a, exit_distance, pocket_collision))
          {continue;}
          // A safe angled corridor may be the first stage of a detour. Do
          // not require the still-blocked goal-facing turn at this position.
          // The next full route is planned/validated AFTER this stopped turn.
          return {true, a};
        }
        return {};
      };
    const auto current_exit = force_retreat ? Exit{} : useful_exit(x, y,
      mode == "forward" || mode == "forward_complete");
    if (mode == "lost_exit") {
      // Only a fresh geometric handoff failure permits another retreat.
      // A native Spin timeout/TF/unknown error must still stop the tree.
      if (getInput<uint16_t>("motion_error").value() != 0 ||
        (!data_->turn_handoff && !data_->turn_geometric_stop) ||
        (current_exit.clear && !data_->turn_geometric_stop) || !rear_clear())
      {return BT::NodeStatus::FAILURE;}
      data_->escape_memory.commit();
      data_->turn_geometric_stop = false;
      data_->turn_handoff = false;
      ++data_->motion_revision;
      RCLCPP_WARN(node_->get_logger(),
        "Turning pocket lost during braking; rear checked, continue seeking different space");
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "forward_begin") {
      const double d = getInput<double>("candidate_distance").value();
      if (!std::isfinite(d) || d < 0.15 || d > 1.20) {
        return BT::NodeStatus::FAILURE;
      }
      if (force_retreat || data_->escape_memory.committed ||
        !forward_at(x, y, yaw, d, departure_collision) ||
        !useful_exit(x + d * std::cos(yaw), y + d * std::sin(yaw), true).clear)
      {
        data_->forward_geometric_stop = true;
        return BT::NodeStatus::FAILURE;
      }
      data_->forward_active = true;
      data_->forward_geometric_stop = false;
      data_->forward_goal = goal;
      data_->forward_x = odom_x; data_->forward_y = odom_y;
      data_->forward_yaw = yaw + ta; data_->forward_distance = d;
      ++data_->motion_revision;
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "forward_safe") {
      if (!data_->forward_active) {return BT::NodeStatus::FAILURE;}
      const double travelled = (odom_x - data_->forward_x) * std::cos(data_->forward_yaw) +
        (odom_y - data_->forward_y) * std::sin(data_->forward_yaw);
      const double remaining = std::clamp(data_->forward_distance - travelled, 0.0, 1.20);
      const bool fits = !force_retreat &&
        forward_at(x, y, yaw, remaining, departure_collision) &&
        useful_exit(x + remaining * std::cos(yaw), y + remaining * std::sin(yaw), true).clear;
      if (!fits) {
        data_->forward_geometric_stop = true;
        ++data_->motion_revision;
      }
      cached_mode_ = mode;
      cached_revision_ = data_->motion_revision;
      cached_time_ = now;
      cached_status_ = fits ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return cached_status_;
    }
    if (mode == "forward_complete") {
      if (!data_->forward_active) {return BT::NodeStatus::FAILURE;}
      if (!current_exit.clear) {
        data_->forward_geometric_stop = true;
        ++data_->motion_revision;
        return BT::NodeStatus::FAILURE;
      }
      data_->forward_active = false;
      data_->escape_memory.noteAdjustment(odom_x, odom_y);
      ++data_->motion_revision;
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "forward_collision") {
      const auto error = getInput<uint16_t>("motion_error").value();
      if ((error != 0 && error != nav2_msgs::action::DriveOnHeading::Result::COLLISION_AHEAD) ||
        (!data_->forward_geometric_stop &&
        error != nav2_msgs::action::DriveOnHeading::Result::COLLISION_AHEAD))
      {return BT::NodeStatus::FAILURE;}
      // Unlike an actually collided spin, a failed advance does not impose
      // an arbitrary retreat distance before accepting a NEW safe turn.
      data_->forward_active = data_->forward_geometric_stop = false;
      data_->escape_memory.commit();
      data_->turn_collision = true;
      data_->collision_min_retreat = 0.0;
      data_->failed_forward = true;
      data_->collision_goal = goal;
      data_->collision_x = odom_x; data_->collision_y = odom_y;
      data_->collision_yaw = yaw + ta;
      ++data_->motion_revision;
      const bool rear = rear_clear();
      RCLCPP_WARN(node_->get_logger(), "Forward escape blocked: %s",
        rear ? "checked rear corridor, switching to continuous retreat" : "rear blocked, stopping");
      return rear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    double advance = 0.0;
    // Do not advance away from an already usable turn. Search actual turning
    // pockets, not arbitrary increments of clear floor. Every intervening
    // footprint and the destination turn/exit are checked before driving.
    if (mode == "forward" && !force_retreat && !data_->escape_memory.committed &&
      (translation_required || !current_exit.clear || current_exit.turn == 0.0))
    {
      const double goal_distance = std::hypot(goal.pose.position.x - x,
        goal.pose.position.y - y);
      for (double d : {0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20}) {
        if (d > goal_distance + 0.04) {continue;}
        if (!forward_at(x, y, yaw, d, departure_collision)) {break;}
        if (useful_exit(x + d * std::cos(yaw), y + d * std::sin(yaw), true).clear) {
          advance = d; break;
        }
      }
    }
    if (mode == "forward") {
      if (advance == 0.0) {return BT::NodeStatus::FAILURE;}
      setOutput("forward_distance", advance);
      RCLCPP_INFO(node_->get_logger(),
        "Validated forward turning/exit pocket %.2f m; not reversing", advance);
      return BT::NodeStatus::SUCCESS;
    }
    const double turn = current_exit.turn;
    if (mode == "turn_begin") {
      if (translation_required) {return BT::NodeStatus::FAILURE;}
      const double a = getInput<double>("candidate_angle").value();
      data_->turn_geometric_stop = false;
      if (!std::isfinite(a) || std::abs(a) < 0.10 || std::abs(a) > 2.0 * kPi + 1e-6) {
        return BT::NodeStatus::FAILURE;
      }
      const double desired = wrapAngle(std::atan2(goal.pose.position.y - y,
        goal.pose.position.x - x) - yaw);
      if (force_retreat ||
        (data_->escape_memory.committed && std::abs(wrapAngle(desired - a)) > 0.10) ||
        !turn_fits(x, y, yaw, a) ||
        !forward_at(x, y, yaw + a, std::min(0.75,
        std::hypot(goal.pose.position.x - x, goal.pose.position.y - y)), pocket_collision))
      {data_->turn_geometric_stop = true; return BT::NodeStatus::FAILURE;}
      data_->selected_turn_goal_facing = std::abs(wrapAngle(desired - a)) <= 0.10;
      data_->escape_memory.noteAdjustment(odom_x, odom_y);
      data_->turn_attempts.push_back({odom_x, odom_y, wrapAngle(yaw + ta + a)});
      if (data_->turn_attempts.size() > 32) {
        data_->turn_attempts.erase(data_->turn_attempts.begin());
      }
      ++data_->motion_revision;
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "turn") {
      if (translation_required || turn == 0.0) {return BT::NodeStatus::FAILURE;}
      setOutput("turn_angle", turn);
      return BT::NodeStatus::SUCCESS;
    }
    // While retreating, NEVER stop just because advancing back toward the
    // previous dead end is possible. Stop only at a useful exit HERE.
    const bool escape = current_exit.clear && !translation_required;
    const bool retreat_clear = !escape && rear_clear();
    if (mode == "reverse_needed" && !escape && !retreat_clear) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "Retreat stopped: no checked exit and rear footprint corridor blocked at (%.3f, %.3f), yaw %.1f deg",
        x, y, yaw * 180.0 / kPi);
    }
    if (mode == "reverse_needed") {
      if (retreat_clear && !data_->retreat_active) {
        data_->turn_handoff = false;
        data_->escape_memory.commit();
        data_->escape_memory.beginReverse(odom_x, odom_y, yaw + ta);
        data_->retreat_active = true;
        data_->retreat_stopped = false;
        data_->retreat_goal = goal;
        data_->retreat_x = odom_x;
        data_->retreat_y = odom_y;
        data_->retreat_yaw = yaw + ta;
        ++data_->motion_revision;
        RCLCPP_INFO(node_->get_logger(),
          "Continuous retreat: keep backing until the complete turn and exit corridor fit");
      } else if (!retreat_clear && data_->retreat_active) {
        // The ReactiveSequence halts BackUp before the fallback replans.
        data_->retreat_active = false;
        data_->retreat_stopped = true;
        ++data_->motion_revision;
        if (escape) {
          RCLCPP_INFO(node_->get_logger(),
            "Checked exit reached: brake, recheck and turn %.1f degrees", turn * 180.0 / kPi);
        }
      }
    } else if (mode == "escape_available" && escape) {
      data_->turn_handoff = true;
      // An existing successful exit already sends the tree back to planning.
      data_->retreat_active = data_->retreat_stopped = false;
      ++data_->motion_revision;
    }
    cached_mode_ = mode;
    cached_revision_ = data_->motion_revision;
    cached_time_ = now;
    cached_status_ = ((mode == "escape_available" && escape) ||
      (mode == "reverse_needed" && retreat_clear)) ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    return cached_status_;
  }

  bool fresh(const builtin_interfaces::msg::Time & stamp, double limit) const
  {
    const double age = (node_->now() - rclcpp::Time(stamp)).seconds();
    return age >= -0.1 && age < limit;
  }

  static std::unique_ptr<nav2_costmap_2d::Costmap2D> makeMap(
    const nav2_msgs::msg::Costmap & msg)
  {
    const auto & m = msg.metadata;
    if (m.size_x == 0 || m.size_y == 0 || m.resolution <= 0.0 ||
      msg.data.size() != static_cast<size_t>(m.size_x) * m.size_y)
    {
      throw std::runtime_error("Invalid costmap dimensions");
    }
    auto map = std::make_unique<nav2_costmap_2d::Costmap2D>(
      m.size_x, m.size_y, m.resolution, m.origin.position.x, m.origin.position.y);
    std::copy(msg.data.begin(), msg.data.end(), map->getCharMap());
    return map;
  }

  static bool occupied(
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & checker,
    double x, double y, double yaw, const nav2_costmap_2d::Footprint & body)
  {
    const double cost = checker.footprintCostAtPose(x, y, yaw, body);
    if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
    unsigned int mx, my;
    if (!checker.worldToMap(x, y, mx, my)) {return true;}
    auto * map = checker.getCostmap();
    if (map->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
    // Nav2's edge checker alone can miss a small obstacle entirely inside
    // the forks/body. Check occupied cell centres inside the polygon too.
    nav2_costmap_2d::Footprint polygon;
    double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
    double max_x = -min_x, max_y = -min_x;
    for (const auto & p : body) {
      geometry_msgs::msg::Point q;
      q.x = x + std::cos(yaw) * p.x - std::sin(yaw) * p.y;
      q.y = y + std::sin(yaw) * p.x + std::cos(yaw) * p.y;
      polygon.push_back(q);
      min_x = std::min(min_x, q.x); max_x = std::max(max_x, q.x);
      min_y = std::min(min_y, q.y); max_y = std::max(max_y, q.y);
    }
    unsigned int ix0, iy0, ix1, iy1;
    if (!map->worldToMap(min_x, min_y, ix0, iy0) ||
      !map->worldToMap(max_x, max_y, ix1, iy1)) {return true;}
    for (unsigned int ix = ix0; ix <= ix1; ++ix) {
      for (unsigned int iy = iy0; iy <= iy1; ++iy) {
        if (map->getCost(ix, iy) < nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
        double wx, wy;
        map->mapToWorld(ix, iy, wx, wy);
        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
          const auto & a = polygon[i]; const auto & b = polygon[j];
          if ((a.y > wy) != (b.y > wy) &&
            wx < (b.x - a.x) * (wy - a.y) / (b.y - a.y) + a.x)
          {
            inside = !inside;
          }
        }
        if (inside) {return true;}
      }
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ClearanceData> data_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_msgs::msg::Costmap::ConstSharedPtr global_, local_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;
  std::string cached_mode_;
  unsigned cached_revision_{0};
  std::chrono::steady_clock::time_point cached_time_;
  BT::NodeStatus cached_status_{BT::NodeStatus::FAILURE};
};

// Search while stopped, rank complete body-checked routes, revalidate them
// at execution time, and try the next route after a geometric control failure.
// Children: proposal/planning/validation, fresh selected-route check, execution.
class ForwardRouteSearch final : public BT::ControlNode
{
public:
  using BT::ControlNode::ControlNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<nav_msgs::msg::Path>("candidate_path"),
      BT::InputPort<std::string>("candidate_planner"),
      BT::InputPort<uint16_t>("controller_error", uint16_t{0}, "Execution fault"),
      BT::InputPort<double>("search_timeout", 60.0, "Stopped planning deadline in seconds"),
      BT::InputPort<bool>("forward_search", true, "Search checked forward departures"),
      BT::InputPort<bool>("compare_planners", false, "Compare all planners at each departure"),
      BT::InputPort<bool>("selection_only", false, "Return shortest route without executing"),
      BT::InputPort<bool>("refresh_on_failure", false, "Request fresh routes before stale detours"),
      BT::InputPort<bool>("explore_forward_detours", false,
        "Compare forward departures before accepting a long current-pose detour"),
      BT::InputPort<bool>("explore_goal_approaches", false,
        "Compare aligned terminal approaches when current-pose routes fail or detour"),
      BT::InputPort<double>("max_near_goal_length", 0.0,
        "Defer longer routes within 25 cm of the goal until checked local escape is tried"),
      BT::OutputPort<std::string>("candidate_departure_mode"),
      BT::OutputPort<std::string>("candidate_planner_request"),
      BT::OutputPort<double>("candidate_distance"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<std::string>("planner_id"),
      BT::OutputPort<double>("local_check_distance")};
  }
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 3) {throw BT::RuntimeError("ForwardRouteSearch needs 3 children");}
    // Nav2 updates the goal blackboard during action preemption. Cancel both
    // an old planner and a running controller before comparing the new goal.
    if (auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal")) {
      const auto & p = goal.value();
      if (goal_seen_ && (p.header.frame_id != goal_.header.frame_id ||
        std::hypot(p.pose.position.x - goal_.pose.position.x,
        p.pose.position.y - goal_.pose.position.y) > 1e-6 ||
        std::abs(wrapAngle(tf2::getYaw(p.pose.orientation) -
        tf2::getYaw(goal_.pose.orientation))) > 1e-6))
      {
        resetChildren();
        resetSearch();
      }
      goal_ = p;
      if (!goal_seen_) {
        RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
          "Route goal: x=%.6f y=%.6f yaw=%.6f rad", p.pose.position.x,
          p.pose.position.y, tf2::getYaw(p.pose.orientation));
      }
      goal_seen_ = true;
    }
    if (deadline_ == std::chrono::steady_clock::time_point{}) {
      const double timeout = getInput<double>("search_timeout").value();
      if (!std::isfinite(timeout) || timeout <= 0.0 || timeout > 120.0) {
        return finish(BT::NodeStatus::FAILURE);
      }
      deadline_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout));
    }
    const bool forward = getInput<bool>("forward_search").value();
    const bool compare = getInput<bool>("compare_planners").value();
    const size_t planners_per_start = compare ? 3 : 1;
    constexpr size_t current_count = 6;
    constexpr size_t forward_count = 8 * 3;
    constexpr size_t approach_begin = current_count + forward_count;
    if (!forward && phase_ == Phase::SEARCH && proposal_ == current_count &&
      getInput<bool>("explore_forward_detours").value())
    {
      double shortest = std::numeric_limits<double>::infinity();
      double direct_distance = 0.0;
      for (const auto & choice : choices_) {
        if (choice.length < shortest) {
          shortest = choice.length;
          const auto & start = choice.path.poses.front().pose.position;
          const auto & end = choice.path.poses.back().pose.position;
          direct_distance = std::hypot(end.x - start.x, end.y - start.y);
        }
      }
      // A large Dubins loop can be an artefact of the departure heading.
      // Compare complete checked forward departures before issuing motion.
      // A genuine obstacle detour remains eligible if no shorter route fits.
      search_forward_ = choices_.empty() ||
        (direct_distance > 0.05 && shortest > 1.35 * direct_distance + 0.50);
      if (search_forward_) {
        RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
          "Current-pose route is a detour (%.3f m vs %.3f m direct); compare forward starts before moving",
          shortest, direct_distance);
      }
      if (getInput<bool>("explore_goal_approaches").value()) {
        if (choices_.empty()) {
          search_approach_ = true;
        }
        // A fully checked Direct route already includes its terminal sweep.
        // Only search standoff approaches when no route from this pose fits;
        // otherwise the extra planning load delays motion and can prefer a
        // longer arrival around empty floor.
        if (search_approach_ && !search_forward_) {
          // Compare aligned arrivals without paying for unrelated forward
          // departure trials on a short, already-clear current-pose route.
          proposal_ = approach_begin;
        }
      }
    }
    const size_t proposal_count = forward ? 8 * planners_per_start :
      (search_approach_ ? approach_begin + 9 :
      (search_forward_ ? approach_begin : current_count));
    setStatus(BT::NodeStatus::RUNNING);
    if (phase_ == Phase::SEARCH) {
      if (std::chrono::steady_clock::now() >= deadline_) {
        haltChild(0);
        proposal_ = proposal_count;
        RCLCPP_WARN(rclcpp::get_logger("forward_route_search"),
          "Forward planning deadline reached; only validated candidates may execute");
      }
      if (proposal_ < proposal_count) {
        const bool approach_proposal = !forward && proposal_ >= approach_begin;
        const bool forward_proposal = !approach_proposal &&
          (forward || proposal_ >= current_count);
        const size_t forward_index = forward ? proposal_ :
          (proposal_ >= current_count ? proposal_ - current_count : 0);
        const size_t count_per_start = forward ? planners_per_start : 3;
        const double departure = approach_proposal ?
          .25 * ((proposal_ - approach_begin) / 3 + 1) :
          (forward_proposal ? 0.15 * (forward_index / count_per_start + 1) : 0.0);
        const std::vector<std::string> planners = {"GridBased", "SE2Arc", "SE2Fallback"};
        const std::vector<std::string> current_planners = {
          "Direct", "DirectArc", "GridShortest", "GridBased", "SE2Arc", "SE2Fallback"};
        setOutput("candidate_distance", departure);
        setOutput("candidate_departure_mode", std::string(approach_proposal ? "approach" :
          (forward_proposal ? "forward" : "current")));
        setOutput("candidate_planner_request", approach_proposal ?
          planners[(proposal_ - approach_begin) % 3] :
          (forward_proposal ?
          planners[forward_index % count_per_start] :
          current_planners[proposal_]));
        const auto result = children_nodes_[0]->executeTick();
        if (result == BT::NodeStatus::RUNNING) {return result;}
        bool direct_is_xy_lower_bound = false;
        bool direct_departure_clear = false;
        if (result == BT::NodeStatus::SUCCESS) {
          auto path = getInput<nav_msgs::msg::Path>("candidate_path").value();
          std::vector<TrackingPose> poses;
          for (const auto & p : path.poses) {
            poses.push_back({p.pose.position.x, p.pose.position.y,
                tf2::getYaw(p.pose.orientation)});
          }
          double length = 0.0;
          for (size_t i = 1; i < poses.size(); ++i) {
            length += std::hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
          }
          const double score = routePreference(poses);
          if (std::isfinite(score)) {
            const auto planner = getInput<std::string>("candidate_planner").value();
            direct_departure_clear = forward_proposal && planner == "Direct";
            if (!forward && !forward_proposal && !approach_proposal &&
              proposal_ == 0 && goal_seen_ &&
              (planner == "Direct" || planner == "DirectReverse") &&
              path.poses.size() >= 2 && path.header.frame_id == goal_.header.frame_id)
            {
              const auto & start = path.poses.front().pose.position;
              const auto & end = path.poses.back().pose.position;
              const double lower_bound = std::hypot(
                goal_.pose.position.x - start.x, goal_.pose.position.y - start.y);
              direct_is_xy_lower_bound =
                std::hypot(end.x - goal_.pose.position.x,
                end.y - goal_.pose.position.y) < .005 &&
                std::abs(length - lower_bound) < .01;
            }
            choices_.push_back({std::move(path),
                planner, score, length,
                std::max(1.0, departure + 0.05)});
            RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
              "Checked candidate: departure %.2f m, planner %s, length %.3f m, motion score %.3f m",
              departure, choices_.back().planner.c_str(), length, score);
          }
        }
        haltChild(0);
        // A body-checked direct segment equals the Euclidean lower bound.
        // No longer candidate can improve XY distance; the fresh-route check
        // still runs before FollowPath and on every BT tick while moving.
        if (direct_is_xy_lower_bound) {
          proposal_ = current_count;
          RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
            "Checked direct route reaches XY lower bound; skip longer planner trials");
        } else if (direct_departure_clear) {
          // With this fixed departure, the checked straight remainder is
          // already the XY lower bound. Compare the next pocket, avoiding
          // duplicate checks and planner calls for longer curves here.
          proposal_ += count_per_start - forward_index % count_per_start;
        } else {
          ++proposal_;
        }
        return BT::NodeStatus::RUNNING;
      }
      std::stable_sort(choices_.begin(), choices_.end(),
        [](const Choice & a, const Choice & b) {return a.score < b.score;});
      const double near_goal_cap = getInput<double>("max_near_goal_length").value();
      if (std::isfinite(near_goal_cap) && near_goal_cap > 0.0) {
        const auto old_count = choices_.size();
        choices_.erase(std::remove_if(choices_.begin(), choices_.end(),
          [&](const Choice & choice) {
            const auto & start = choice.path.poses.front().pose.position;
            return std::hypot(goal_.pose.position.x - start.x,
                     goal_.pose.position.y - start.y) < 0.25 &&
                   choice.length > near_goal_cap;
          }), choices_.end());
        if (old_count != choices_.size()) {
          RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
            "Deferred %zu long near-goal routes until checked local escape",
            old_count - choices_.size());
        }
      }
      RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
        "Route search found %zu complete body-checked candidates; lowest motion score first",
        choices_.size());
      phase_ = Phase::VALIDATE;
    }
    if (selected_ >= choices_.size()) {return finish(BT::NodeStatus::FAILURE);}
    if (phase_ == Phase::VALIDATE) {
      setOutput("path", choices_[selected_].path);
      setOutput("planner_id", choices_[selected_].planner);
      setOutput("local_check_distance", choices_[selected_].local_range);
      const auto result = children_nodes_[1]->executeTick();
      if (result == BT::NodeStatus::RUNNING) {return result;}
      haltChild(1);
      if (result != BT::NodeStatus::SUCCESS) {
        ++selected_;
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::EXECUTE;
      // ComputePathToPose publishes each trial to /plan. RViz must show the
      // checked winner actually sent to FollowPath, including Direct paths.
      std::shared_ptr<ClearanceData> data;
      if (config().blackboard->get("tai_clearance_data", data)) {
        auto chosen = choices_[selected_].path;
        chosen.header.stamp = data->node->now();
        data->selected_route_pub->publish(chosen);
      }
      RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
        "Trying ranked route %zu/%zu, planner %s, length %.3f m",
        selected_ + 1, choices_.size(), choices_[selected_].planner.c_str(),
        choices_[selected_].length);
      if (getInput<bool>("selection_only").value()) {return finish(BT::NodeStatus::SUCCESS);}
    }
    const auto result = children_nodes_[2]->executeTick();
    if (result == BT::NodeStatus::RUNNING) {return result;}
    haltChild(2);
    if (result == BT::NodeStatus::SUCCESS) {
      std::shared_ptr<ClearanceData> data;
      if (goal_seen_ && config().blackboard->get("tai_clearance_data", data)) {
        // RecoveryNode normally repeats its primary after a recovery succeeds.
        // Here the recovery may itself have completed FollowPath to the goal.
        // Preserve that authoritative result instead of starting another action.
        data->completed_goal = goal_;
        data->completed_goal_valid = true;
      }
      return finish(result);
    }
    const auto error = getInput<uint16_t>("controller_error").value();
    if (error != 0 && error != 104 && error != 105 && error != 106) {
      RCLCPP_WARN(rclcpp::get_logger("forward_route_search"),
        "Controller fault %u: stopping route retries", error);
      return finish(BT::NodeStatus::FAILURE);
    }
    if (getInput<bool>("refresh_on_failure").value()) {
      RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
        "Selected route stopped; compare fresh routes from the actual pose before longer detours");
      return finish(BT::NodeStatus::FAILURE);
    }
    RCLCPP_WARN(rclcpp::get_logger("forward_route_search"),
      "Forward route execution stopped (controller %u); checking next ranked route", error);
    ++selected_;
    phase_ = Phase::VALIDATE;
    return BT::NodeStatus::RUNNING;
  }
  void halt() override
  {
    BT::ControlNode::halt();
    resetSearch();
  }
private:
  BT::NodeStatus finish(BT::NodeStatus result)
  {
    resetChildren();
    resetSearch();
    return result;
  }
  void resetSearch()
  {
    proposal_ = selected_ = 0;
    phase_ = Phase::SEARCH;
    choices_.clear();
    deadline_ = {};
    goal_seen_ = false;
    search_forward_ = false;
    search_approach_ = false;
  }
  struct Choice {
    nav_msgs::msg::Path path;
    std::string planner;
    double score, length, local_range;
  };
  enum class Phase {SEARCH, VALIDATE, EXECUTE};
  Phase phase_{Phase::SEARCH};
  size_t proposal_{0}, selected_{0};
  std::vector<Choice> choices_;
  std::chrono::steady_clock::time_point deadline_;
  geometry_msgs::msg::PoseStamped goal_;
  bool goal_seen_{false}, search_forward_{false}, search_approach_{false};
};

// Asynchronously prime the shared cache before ANY planner/controller/recovery
// executes. Never turn "no callback received yet" into a geometric collision.
class WaitForClearanceData final : public BT::StatefulActionNode
{
public:
  WaitForClearanceData(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config)
  {
    auto readiness = config;
    readiness.input_ports["data_only"] = "true";
    checker_ = std::make_unique<PathFootprintClear>(name + "_check", readiness);
  }
  static BT::PortsList providedPorts() {return PathFootprintClear::providedPorts();}
  BT::NodeStatus onStart() override
  {
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    if (checker_->executeTick() == BT::NodeStatus::SUCCESS) {return BT::NodeStatus::SUCCESS;}
    return std::chrono::steady_clock::now() < deadline_ ?
           BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
  void onHalted() override {checker_->halt();}

private:
  std::unique_ptr<PathFootprintClear> checker_;
  std::chrono::steady_clock::time_point deadline_;
};

class ResetEscapeState final : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("reset_motion_history", false, "New goal execution"),
      BT::InputPort<bool>("turn_completed", false, "A checked Spin actually succeeded"),
      BT::InputPort<bool>("invalidate_path", false, "Require planning from the new pose")};
  }
  BT::NodeStatus tick() override
  {
    if (getInput<bool>("turn_completed").value()) {
      std::shared_ptr<ClearanceData> data;
      if (config().blackboard->get("tai_clearance_data", data)) {
        data->failed_forward = data->turn_collision = false;
        if (data->escape_memory.committed && data->selected_turn_goal_facing) {
          data->escape_memory.reset();
        }
        data->selected_turn_goal_facing = data->turn_geometric_stop = false;
        data->turn_handoff = false;
        ++data->motion_revision;
      }
    }
    if (getInput<bool>("reset_motion_history").value()) {
      std::shared_ptr<ClearanceData> data;
      if (config().blackboard->get("tai_clearance_data", data)) {
        data->completed_goal_valid = false;
        data->turn_collision = false;
        data->failed_forward = false;
        data->retreat_active = data->retreat_stopped = false;
        data->near_goal_reverse_pending = data->near_goal_reverse_done = false;
        data->terminal_escape_needed = false;
        data->forward_active = data->forward_geometric_stop = false;
        data->turn_attempts.clear();
        data->escape_memory.reset();
        data->selected_turn_goal_facing = data->turn_geometric_stop = false;
        data->turn_handoff = false;
        ++data->motion_revision;
      }
    }
    config().blackboard->set<uint16_t>("compute_path_error_code", 0);
    config().blackboard->set<uint16_t>("follow_path_error_code", 0);
    config().blackboard->set<uint16_t>("backup_error_code", 0);
    config().blackboard->set<uint16_t>("escape_forward_error", 0);
    config().blackboard->set<uint16_t>("escape_turn_error", 0);
    config().blackboard->set<bool>("local_escape_required", false);
    config().blackboard->set<std::string>("path_planner", "SE2Fallback");
    if (getInput<bool>("invalidate_path").value()) {
      config().blackboard->set("path", nav_msgs::msg::Path{});
    }
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace tai_robot_one

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tai_robot_one::PathFootprintClear>("PathFootprintClear");
  factory.registerNodeType<tai_robot_one::ForwardRouteSearch>("ForwardRouteSearch");
  factory.registerNodeType<tai_robot_one::ForwardRouteSearch>("RankedRouteSearch");
  factory.registerNodeType<tai_robot_one::ResetEscapeState>("ResetEscapeState");
  factory.registerNodeType<tai_robot_one::WaitForClearanceData>("WaitForClearanceData");
}
