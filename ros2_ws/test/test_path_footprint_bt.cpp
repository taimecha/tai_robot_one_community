// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <tuple>
#include <nlohmann/json.hpp>
#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FootprintBTTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // Isolate synthetic costmaps from the real robot's ROS graph.
    setenv("ROS_DOMAIN_ID", "231", 1);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", false)});
    node = std::make_shared<rclcpp::Node>("footprint_bt_test", options);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    board = BT::Blackboard::create();
    board->set("node", node);
    board->set("tf_buffer", tf);
    factory.registerFromPlugin(FOOTPRINT_BT_LIBRARY);
    tree = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
      "<PathFootprintClear path='{path}'/></BehaviorTree></root>", board);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    global = node->create_publisher<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", qos);
    local = node->create_publisher<nav2_msgs::msg::Costmap>(
      "/local_costmap/costmap_raw", qos);
    footprint = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/local_costmap/published_footprint", rclcpp::QoS(1).reliable());
    map.header.frame_id = "map";
    map.metadata.size_x = map.metadata.size_y = 160;
    map.metadata.resolution = 0.025;
    map.metadata.origin.position.x = map.metadata.origin.position.y = -2.0;
    map.metadata.origin.orientation.w = 1.0;
    map.data.assign(160 * 160, 0);
    body.header.frame_id = "base_footprint";
    for (auto xy : {std::pair<double, double>{0.81, 0.13}, {0.81, -0.13},
        {0.41, -0.28}, {-0.36, -0.28}, {-0.36, 0.28}, {0.41, 0.28}})
    {
      geometry_msgs::msg::Point32 p;
      p.x = xy.first; p.y = xy.second;
      body.polygon.points.push_back(p);
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (double x : {0.0, 0.9}) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = x;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    board->set("path", path);
  }

  void publish()
  {
    const auto now = node->now();
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = split_frames ? "odom" : "map";
    t.child_frame_id = "base_footprint";
    t.transform = robot_transform;
    tf->setTransform(t, "test", false);
    if (split_frames) {
      geometry_msgs::msg::TransformStamped localization;
      localization.header.frame_id = "map";
      localization.child_frame_id = "odom";
      localization.header.stamp = now - rclcpp::Duration::from_seconds(5.0);
      localization.transform = localization_transform;
      tf->setTransform(localization, "test", false);
    }
    map.header.stamp = now;
    body.header.stamp = now;
    local_map.header.stamp = now;
    global->publish(map);
    local->publish(split_frames ? local_map : map);
    footprint->publish(body);
  }

  void connect()
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < until) {
      publish();
      tree.tickOnce();
      if (footprint->get_subscription_count() > 0 &&
        global->get_subscription_count() > 0 && local->get_subscription_count() > 0)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(30));
        publish();
        rclcpp::sleep_for(std::chrono::milliseconds(30));
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    FAIL() << "Synthetic costmap publishers did not discover BT subscriptions";
  }

  void selectMotion(const std::string & motion)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 1.5;
    goal.pose.orientation.w = 1.0;
    board->set("goal", goal);
    tree = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
      "<PathFootprintClear motion='" + motion + "' goal='{goal}' "
      "forward_distance='{distance}' turn_angle='{angle}' planner_error='{planner}' "
      "controller_error='{controller}' local_blocked='{blocked}' motion_error='{motion_error}'/>"
      "</BehaviorTree></root>", board);
    board->set<uint16_t>("planner", 0);
    board->set<uint16_t>("controller", 0);
    board->set("blocked", false);
    board->set<uint16_t>("motion_error", 0);
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  BT::Blackboard::Ptr board;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  nav2_msgs::msg::Costmap map;
  nav2_msgs::msg::Costmap local_map;
  bool split_frames{false};
  geometry_msgs::msg::Transform robot_transform = [] {
      geometry_msgs::msg::Transform t; t.rotation.w = 1.0; return t;
    }();
  geometry_msgs::msg::Transform localization_transform = [] {
      geometry_msgs::msg::Transform t; t.rotation.w = 1.0; return t;
    }();
  geometry_msgs::msg::PolygonStamped body;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr global, local;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint;
};

class RunningBackup : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts() {return {};}
  static inline int starts = 0;
  static inline int stops = 0;
  BT::NodeStatus onStart() override {++starts; return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {++stops;}
};

class CollisionSpin : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    config().blackboard->set<uint16_t>("motion_error", 703);
    return BT::NodeStatus::FAILURE;
  }
};

TEST_F(FootprintBTTest, AcceptsClearRouteWithFullForkFootprint)
{
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, GoalApproachJoinsAlignedCorridorAndExactDestination)
{
  geometry_msgs::msg::PoseStamped goal, approach;
  goal.header.frame_id = approach.header.frame_id = "map";
  goal.pose.position.x = .9;
  goal.pose.orientation.w = 1;
  approach = goal;
  approach.pose.position.x = .4;
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.back().pose.position.x = .4;
  board->set("path", path);
  board->set("goal", goal);
  board->set("approach", approach);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='approach_route' path='{path}' goal='{goal}' "
    "candidate_start='{approach}' checked_path='{joined}'/></BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto joined = board->get<nav_msgs::msg::Path>("joined");
  EXPECT_DOUBLE_EQ(joined.poses.back().pose.position.x, .9);
  ASSERT_GT(joined.poses.size(), 4u);
  EXPECT_DOUBLE_EQ(joined.poses[1].pose.position.x, .4);
  EXPECT_DOUBLE_EQ(joined.poses[2].pose.position.x, .4);
  map.data[80 * 160 + 136] = 254;  // x=1.4125, in the final fork corridor
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, TerminalNearestPointApproachesBeforeRotatingGoalYaw)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.front().pose.position.x = -.5;
  path.poses.back().pose.position.x = .12;
  path.poses.back().pose.orientation.z = std::sin(2.3 / 2);
  path.poses.back().pose.orientation.w = std::cos(2.3 / 2);
  board->set("path", path);
  // A pole intersects the yaw sweep at x=0, but not at the exact goal x=.12.
  map.data[106 * 160 + 63] = 254;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, PreparedArrivalAlignsAtAcceptedXYWithoutExtraTranslation)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.back().pose.position.x = .04;
  board->set("path", path);
  map.data[80 * 160 + 113] = 254;  // x=.8375: exact endpoint blocked, current body clear
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = .08;  // still within tolerance, but real body blocked
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, SlightlyFutureFootprintStampDoesNotCancelClearRoute)
{
  connect();
  body.header.frame_id = "map";
  body.header.stamp = node->now() + rclcpp::Duration::from_seconds(0.07);
  footprint->publish(body);
  rclcpp::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, TerminalPivotCanLeaveOnlyExistingGlobalPaddingContact)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.back().pose.position.x = .04;
  path.poses.back().pose.orientation.z = std::sin(-.5 / 2);
  path.poses.back().pose.orientation.w = std::cos(-.5 / 2);
  board->set("path", path);
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  map.data[85 * 160 + 112] = 254;  // Raster fork edge, outside the original core.
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto goal = path.poses.back();
  goal.header.frame_id = "map";
  board->set("goal", goal);
  auto direct_tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{direct_path}'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(direct_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // A newly encountered global cell during the turn remains blocking.
  map.data[66 * 160 + 110] = 254;
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  map.data[66 * 160 + 110] = 0;
  // Live contact must not be treated as a map-only padding discrepancy.
  local_map.data[85 * 160 + 112] = 254;
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  local_map.data[85 * 160 + 112] = 0;
  // Even an existing global contact inside the core remains blocking.
  map.data[80 * 160 + 109] = 254;
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, SafeTerminalPivotDoesNotForceLongerAlignedRoute)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.back().pose.orientation.z = std::sin(.5 / 2);
  path.poses.back().pose.orientation.w = std::cos(.5 / 2);
  board->set("path", path);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // A local 20 cm correction also finishes with a checked yaw turn.
  path.poses.front().pose.position.x = .7;
  board->set("path", path);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, RejectsRouteThatCanAlignOnlyAtExactGoalCell)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.front().pose.position.x = .7;  // short correction bypasses long-route rule
  path.poses.back().pose.orientation.z = std::sin(1.2 / 2);
  path.poses.back().pose.orientation.w = std::cos(1.2 / 2);
  board->set("path", path);
  // At x=.85 the first accepted XY has a blocked positive yaw sweep;
  // translating the final 5 cm to exact x=.90 makes that sweep clear.
  map.data[91 * 160 + 106] = 254;  // x=.6625, y=.2875
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, DirectRouteCanApproachExactGoalBeforeFinalPivot)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = .9;
  goal.pose.orientation.z = std::sin(1.2 / 2);
  goal.pose.orientation.w = std::cos(1.2 / 2);
  board->set("goal", goal);
  // Rotation first intersects this cell at the 5 cm XY boundary. The
  // straight final centimetres and the yaw sweep at the exact goal are clear.
  map.data[104 * 160 + 114] = 254;  // x=.8625, y=.6125
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto path = board->get<nav_msgs::msg::Path>("path");
  robot_transform.translation.x = .86;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' planner_id='Direct' prepared='true'/>"
    "</BehaviorTree></root>", board);
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, RejectsTerminalTurnSensitiveToTrackingOffset)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.front().pose.position.x = .7;
  path.poses.back().pose.orientation.z = std::sin(1.2 / 2);
  path.poses.back().pose.orientation.w = std::cos(1.2 / 2);
  board->set("path", path);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // Centered approach clears this cell; a 4 cm lateral tracking error
  // makes the final in-place yaw sweep intersect it before XY acceptance.
  map.data[91 * 160 + 98] = 254;  // x=.4625, y=.2875
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, RejectsOffsetThatWouldNeedBlockedGoalBearingTurn)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  path.poses.back().pose.orientation.z = std::sin(.2 / 2);
  path.poses.back().pose.orientation.w = std::cos(.2 / 2);
  board->set("path", path);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // From a 4 cm offset north of the terminal path, the goal is southeast.
  // Turning toward that bearing reaches this cell although the final +0.2
  // rad yaw sweep and the centered path are both clear.
  map.data[63 * 160 + 130] = 254;  // x=1.2625, y=-.4125
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, NearbyGoalDoesNotBypassRemainingDepartureChecks)
{
  auto path = board->get<nav_msgs::msg::Path>("path");
  auto goal = path.poses.front();
  goal.pose.position.x = .04;
  path.poses.push_back(goal);  // leave to x=.9, return to the nearby goal
  board->set("path", path);
  map.data[80 * 160 + 136] = 254;  // x=1.4125: blocks departure, not current body
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, CloseGoalAllowsCheckedRouteSearchButNotLocalEscape)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = .04;
  goal.pose.orientation.w = 1.0;
  selectMotion("motion_fault_free");
  board->set("goal", goal);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  board->set<uint16_t>("controller", 107);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  board->set<uint16_t>("controller", 0);
  selectMotion("recovery_allowed");
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, RejectsCorridorNarrowerThanBody)
{
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[70 * 160 + ix] = 254;  // y=-0.2375, inside 0.56 m wide body
    map.data[89 * 160 + ix] = 254;
  }
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, DetectsSmallObstacleInsideForkPolygon)
{
  map.data[80 * 160 + 104] = 254;  // x=0.6125, y=0.0125
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, RejectsStaleObstacleData)
{
  connect();
  map.header.stamp = node->now() - rclcpp::Duration::from_seconds(5.0);
  global->publish(map); local->publish(map);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, AcceptsFreshOdometryWithOlderStationaryLocalization)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, AllowsForwardDepartureFromMapOnlyPaddingOverlap)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  map.data[80 * 160 + 65] = 254;  // rear padding cell; no observed local obstacle
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, NeverClearsMapObstacleInsideCoreBody)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  map.data[80 * 160 + 70] = 254;
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, AllowsCheckedForwardExitFromExistingMapOverlap)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  map.data[80 * 160 + 70] = 254;  // existing rear core map overlap
  selectMotion("forward");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(board->get<double>("distance"), 0.15);
}

TEST_F(FootprintBTTest, EscapeNeverIgnoresNewStaticObstacleAhead)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  map.data[80 * 160 + 70] = 254;
  map.data[80 * 160 + 116] = 254;  // x=0.9125: outside starting footprint
  selectMotion("forward");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, EscapeNeverIgnoresObservedLocalObstacle)
{
  map.data[80 * 160 + 70] = 254;
  selectMotion("forward");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, DoesNotReverseWhenTurningOpensForwardExit)
{
  map.data[80 * 160 + 116] = 254;
  selectMotion("turn");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_LT(std::abs(board->get<double>("angle")), 3.15);
  selectMotion("reverse_needed");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, DistantBlockedRouteDoesNotAuthorizeRecoveryMotion)
{
  selectMotion("recovery_allowed");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  board->set<uint16_t>("planner", 206);  // goal occupied
  board->set("blocked", true);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  board->set<uint16_t>("planner", 205);  // start occupied
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, DeadEndAheadRetreatsContinuouslyUntilRearTurningPocket)
{
  // A 62.5 cm corridor permits translation but not a fork/body turn.
  // There is 45 cm of free travel ahead, ending at a wall. Behind x=-.6
  // the corridor opens into a wide turning pocket.
  for (unsigned int ix = 56; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {
    map.data[iy * 160 + 130] = 254;
  }
  selectMotion("forward");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -1.2;
  goal.pose.position.y = 1.0;
  board->set("goal", goal);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "Free translation ahead must not be mistaken for a useful turning pocket";
  factory.registerNodeType<RunningBackup>("RunningBackup");
  RunningBackup::starts = RunningBackup::stops = 0;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><Fallback>"
    "<ReactiveSequence><PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "<RunningBackup/></ReactiveSequence>"
    "<PathFootprintClear motion='escape_available' goal='{goal}'/>"
    "</Fallback></BehaviorTree></root>", board);
  for (double x : {0.0, -0.30, -0.60, -0.90}) {
    robot_transform.translation.x = x;
    rclcpp::sleep_for(std::chrono::milliseconds(220));
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(40));
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
  robot_transform.translation.x = -1.45;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 1);
  selectMotion("turn");
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_GT(std::abs(board->get<double>("angle")), 0.5);
}

TEST_F(FootprintBTTest, AdvancesToUsefulTurningPocketInsteadOfArbitraryShortStep)
{
  for (unsigned int ix = 0; ix <= 92; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  selectMotion("forward");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 1.4;
  goal.pose.position.y = 1.0;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_GT(board->get<double>("distance"), 0.30);
  EXPECT_LE(board->get<double>("distance"), 1.20);
  robot_transform.translation.x = board->get<double>("distance");
  selectMotion("turn");
  board->set("goal", goal);
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, BlockedRearHaltsContinuousRetreatWithoutInventingExit)
{
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {
    map.data[iy * 160 + 130] = 254;
  }
  selectMotion("reverse_needed");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // Add a NEW obstacle 9 cm behind the padded rear: guard must stop.
  map.data[80 * 160 + 61] = 254;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  selectMotion("escape_available");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, SpinCollisionRetreatsFurtherInsteadOfAbortingOrRetryingSameTurn)
{
  selectMotion("turn");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = goal.pose.position.y = 1.0;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  factory.registerNodeType<CollisionSpin>("CollisionSpin");
  factory.registerNodeType<RunningBackup>("RunningBackup");
  RunningBackup::starts = RunningBackup::stops = 0;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><Fallback>"
    "<CollisionSpin/><Sequence>"
    "<PathFootprintClear motion='turn_collision' goal='{goal}' motion_error='{motion_error}'/>"
    "<Fallback><ReactiveSequence>"
    "<PathFootprintClear motion='reverse_needed' goal='{goal}'/><RunningBackup/>"
    "</ReactiveSequence><PathFootprintClear motion='escape_available' goal='{goal}'/>"
    "</Fallback></Sequence></Fallback></BehaviorTree></root>", board);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
  // Replanning errors must not erase the extra-retreat commitment.
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><ResetEscapeState/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -0.10;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  robot_transform.translation.x = -0.20;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 1);
}

TEST_F(FootprintBTTest, SpinTimeoutOrTfFaultNeverAuthorizesReverse)
{
  selectMotion("turn_collision");
  connect();
  for (uint16_t error : {uint16_t{700}, uint16_t{701}, uint16_t{702}}) {
    board->set("motion_error", error);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
  board->set<uint16_t>("motion_error", 703);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  map.data[80 * 160 + 61] = 254;
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "Even a spin collision cannot authorize backing into an obstacle";
}

TEST_F(FootprintBTTest, LocalizationJumpOrForwardMotionDoesNotSatisfyRequiredRetreat)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  selectMotion("turn_collision");
  board->set<uint16_t>("motion_error", 703);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  selectMotion("reverse_needed");
  // A map correction moves the map pose by 30 cm, but odom never moved.
  localization_transform.translation.x = 0.30;
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // Nor does moving forward count as measured reverse travel.
  robot_transform.translation.x = 0.20;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState reset_motion_history='true'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  // A new goal execution releases the old motion commitment immediately,
  // including the guard's cached result.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, SafePartialTurnWithCheckedExitDoesNotRequireWholeGoalFacingSweep)
{
  // The front-right fork hits this cell after ~70 degrees, but a 30-degree
  // partial turn and a short forward corridor still fit the old heuristic.
  map.data[109 * 160 + 89] = 254;
  selectMotion("turn");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0.0;
  goal.pose.position.y = 1.4;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const double partial = board->get<double>("angle");
  EXPECT_GT(std::abs(partial), 0.10);
  EXPECT_LT(std::abs(partial), 1.4);
  selectMotion("reverse_needed");
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  robot_transform.translation.x = -0.30;
  selectMotion("turn");
  board->set("goal", goal);
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(board->get<double>("angle"), std::atan2(1.4, 0.30), 1e-5);
}

TEST_F(FootprintBTTest, ReverseReleasesOnlyMapPaddingContactWithClearLocalAndClearEndpoint)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  map.data[80 * 160 + 112] = 254;  // padded front edge, outside the core body
  selectMotion("reverse_needed");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // A new obstacle behind the actual starting body still forbids release.
  map.data[80 * 160 + 61] = 254;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, ReverseNeverReleasesCoreUnknownOrObservedOverlap)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  selectMotion("reverse_needed");
  map.data[80 * 160 + 104] = 254;  // true core body
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  map.data[80 * 160 + 104] = 0;
  map.data[80 * 160 + 112] = 255;  // unknown padding is not releasable
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  map.data[80 * 160 + 112] = 254;
  local_map.data[80 * 160 + 112] = 254;  // locally observed contact
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, FailedAdvanceRejectsTinyTurnButAcceptsFullGoalFacingTurnImmediately)
{
  selectMotion("forward_collision");
  board->set<uint16_t>("motion_error", 723);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  selectMotion("turn");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  selectMotion("reverse_needed");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0; goal.pose.position.y = 1.4;
  board->set("goal", goal);
  // Commit a failed advance for this new goal, then accept the entire turn
  // immediately because all of its sweep and departure fit here.
  auto collided = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_collision' goal='{goal}' motion_error='723'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(collided.tickOnce(), BT::NodeStatus::SUCCESS);
  auto turn = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn' goal='{goal}' turn_angle='{angle}'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(turn.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(board->get<double>("angle"), 1.57079632679, 1e-5);
}

TEST_F(FootprintBTTest, LostForwardCorridorHaltsAdvanceAndStartsOneContinuousBackup)
{
  selectMotion("forward_begin");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  factory.registerNodeType<RunningBackup>("RunningBackup");
  RunningBackup::starts = RunningBackup::stops = 0;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><Fallback>"
    "<ReactiveSequence><PathFootprintClear motion='forward_safe' goal='{goal}'/>"
    "<AlwaysSuccess/></ReactiveSequence>"
    "<Sequence><PathFootprintClear motion='forward_collision' goal='{goal}'/>"
    "<ReactiveSequence><PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "<RunningBackup/></ReactiveSequence></Sequence></Fallback>"
    "</BehaviorTree></root>", board);
  // Insert a wall ahead after the pocket was accepted; it does not touch
  // the current body but blocks the planned forward translation.
  for (unsigned int iy = 0; iy < 160; ++iy) {map.data[iy * 160 + 115] = 254;}
  // Narrow sides ensure no usable turn here; only the checked rear remains.
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  for (int i = 0; i < 10; ++i) {EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);}
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
}

TEST_F(FootprintBTTest, FailedAdvanceFaultsNeverAuthorizeReverseAndNewGoalClearsStop)
{
  selectMotion("forward_collision");
  connect();
  for (uint16_t error : {uint16_t{0}, uint16_t{720}, uint16_t{721},
      uint16_t{722}, uint16_t{724}})
  {
    board->set("motion_error", error);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
  board->set<uint16_t>("motion_error", 723);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  selectMotion("turn");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState reset_motion_history='true'/></BehaviorTree></root>", board);
  ASSERT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "A new execution must not retain the failed-forward forced-turn state";
}

TEST_F(FootprintBTTest, ExecutedTurnIsNotRepeatedAtSamePositionButTranslationReleasesIt)
{
  map.data[109 * 160 + 89] = 254;
  selectMotion("turn");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0.0; goal.pose.position.y = 1.4;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const double a = board->get<double>("angle");
  board->set("attempt_angle", a);
  auto begin = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn_begin' goal='{goal}' candidate_angle='{attempt_angle}'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(begin.tickOnce(), BT::NodeStatus::SUCCESS);
  auto collided = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn_collision' goal='{goal}' motion_error='703'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(collided.tickOnce(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_GT(std::abs(board->get<double>("angle") - a), 0.12);
  robot_transform.translation.x = -0.30;
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(board->get<double>("angle"), std::atan2(1.4, .30), 1e-5);
}

TEST_F(FootprintBTTest, TwoLocalAdjustmentsSurviveReplansAndPreferContinuousRetreat)
{
  map.data[109 * 160 + 89] = 254;  // full goal-facing turn does not fit here
  selectMotion("turn");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0; goal.pose.position.y = 1.4;
  board->set("goal", goal);
  connect();
  auto begin = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn_begin' goal='{goal}' candidate_angle='{angle}'/>"
    "</BehaviorTree></root>", board);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState invalidate_path='true' turn_completed='true'/>"
    "</BehaviorTree></root>", board);
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(begin.tickOnce(), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "Another tiny turn in the same pocket must not interrupt retreat";
  auto recovery = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='recovery_allowed' goal='{goal}' "
    "planner_error='208' controller_error='0' local_blocked='false'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(recovery.tickOnce(), BT::NodeStatus::SUCCESS)
    << "Local adjustment history must survive an overwritten planner blockage flag";
  auto forward = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward' goal='{goal}'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(forward.tickOnce(), BT::NodeStatus::FAILURE);
  factory.registerNodeType<RunningBackup>("RunningBackup");
  RunningBackup::starts = RunningBackup::stops = 0;
  auto backup = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><ReactiveSequence>"
    "<PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "<RunningBackup/></ReactiveSequence></BehaviorTree></root>", board);
  for (int i = 0; i < 10; ++i) {EXPECT_EQ(backup.tickOnce(), BT::NodeStatus::RUNNING);}
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
  robot_transform.translation.x = -.31;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(backup.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(RunningBackup::stops, 1);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(board->get<double>("angle"), std::atan2(1.4, .31), 1e-5);
}

TEST_F(FootprintBTTest, LostBrakingPocketAllowsRetreatButNotNativeFaultOrBlockedRear)
{
  selectMotion("escape_available");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0; goal.pose.position.y = 1.4;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);  // stop BackUp to brake
  auto lost = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='lost_exit' goal='{goal}' motion_error='{motion_error}'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(lost.tickOnce(), BT::NodeStatus::FAILURE);  // exit still available
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = map.data[92 * 160 + ix] = 254;
  }
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  for (uint16_t code : {uint16_t{700}, uint16_t{701}, uint16_t{702}, uint16_t{704}}) {
    board->set("motion_error", code);
    EXPECT_EQ(lost.tickOnce(), BT::NodeStatus::FAILURE);
  }
  board->set<uint16_t>("motion_error", 0);
  ASSERT_EQ(lost.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(lost.tickOnce(), BT::NodeStatus::FAILURE);  // consumed once
  auto reverse = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(reverse.tickOnce(), BT::NodeStatus::SUCCESS);
  map.data[80 * 160 + 59] = 254;  // x=-.5125, blocks checked rear corridor
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(reverse.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(lost.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, NewGoalClearsRetreatCommitmentWithoutPermittingFaultRecovery)
{
  selectMotion("lost_exit");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "No braking handoff/geometry failure means no extra recovery allowance";
  auto collided = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_collision' goal='{goal}' motion_error='723'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(collided.tickOnce(), BT::NodeStatus::SUCCESS);
  auto candidate = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_candidate' goal='{goal}'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(candidate.tickOnce(), BT::NodeStatus::FAILURE);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 1.6;
  board->set("goal", goal);
  EXPECT_EQ(candidate.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, NearbyStraightGoalUsesExactShortestBodyCheckedPath)
{
  selectMotion("direct");
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 0.25;
  goal.pose.orientation.w = 1.0;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GE(path.poses.size(), 10u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 0.25);
  map.data[80 * 160 + 118] = 254;  // x=.9625: collision with advancing forks
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, ClearOneMetreGoalStaysStraight)
{
  selectMotion("direct");
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.0;
  goal.pose.orientation.w = 1.0;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GT(path.poses.size(), 30u);
  for (const auto & pose : path.poses) {
    EXPECT_NEAR(pose.pose.position.y, 0.0, 1e-6);
  }
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 1.0);
}

TEST_F(FootprintBTTest, GridRouteWithDifferentGoalYawUsesCheckedTerminalPivot)
{
  nav_msgs::msg::Path raw;
  raw.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = "map";
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 0.90;
  goal.pose.orientation.z = std::sin(M_PI / 4.0);
  goal.pose.orientation.w = std::cos(M_PI / 4.0);
  raw.poses = {start, goal};
  board->set("path", raw);
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' goal='{goal}' planner_id='GridShortest' "
    "checked_path='{checked}'/></BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // The line itself remains clear; this cell obstructs the final 90 deg spin.
  const auto ix = static_cast<size_t>((0.875 + 2.0) / 0.025);
  const auto iy = static_cast<size_t>((0.700 + 2.0) / 0.025);
  map.data[iy * 160 + ix] = 254;
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, OffsetGoalOffersCheckedPivotStraightAndFinalTurn)
{
  selectMotion("direct");
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = node->now();
  goal.pose.position.x = 0.85;
  goal.pose.position.y = 0.30;
  goal.pose.orientation.z = std::sin(-0.30);
  goal.pose.orientation.w = std::cos(-0.30);
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GT(path.poses.size(), 30u);
  EXPECT_NEAR(path.poses[1].pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(path.poses[1].pose.position.y, 0.0, 1e-9);
  const double bearing = std::atan2(0.30, 0.85);
  EXPECT_NEAR(tf2::getYaw(path.poses[1].pose.orientation), bearing, 1e-9);
  double length = 0.0;
  for (size_t i = 2; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    EXPECT_NEAR(b.y, b.x * 0.30 / 0.85, 1e-9);
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  EXPECT_NEAR(length, std::hypot(0.85, 0.30), 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), -0.60, 1e-9);
  EXPECT_EQ(path.poses.back().header.stamp.sec, 0);
  EXPECT_EQ(path.poses.back().header.stamp.nanosec, 0u);
}

TEST_F(FootprintBTTest, OpenOffsetGoalGetsTangentCircularArc)
{
  selectMotion("direct_arc");
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 0.7;
  goal.pose.position.y = 0.3;
  const double sweep = 2.0 * std::atan2(0.3, 0.7);
  goal.pose.orientation.z = std::sin(sweep / 2.0);
  goal.pose.orientation.w = std::cos(sweep / 2.0);
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct_arc' goal='{goal}' checked_path='{path}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GT(path.poses.size(), 20u);
  const double radius = (0.7 * 0.7 + 0.3 * 0.3) / (2.0 * 0.3);
  for (const auto & pose : path.poses) {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    EXPECT_NEAR(x * x + (y - radius) * (y - radius), radius * radius, 1e-5);
  }
  EXPECT_LT(std::abs(tf2::getYaw(path.poses[1].pose.orientation)), 0.04);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 0.7);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, 0.3);
  map.data[100 * 160 + 120] = 254;  // (1.0, 0.5), in the swept body
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, WideCornerUsesQuarterCircleWhenFootprintFits)
{
  selectMotion("direct_arc");
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  constexpr double kQuarterTurn = 0.7853981633974483;
  goal.pose.orientation.z = std::sin(kQuarterTurn);
  goal.pose.orientation.w = std::cos(kQuarterTurn);
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct_arc' goal='{goal}' checked_path='{path}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GT(path.poses.size(), 50u);
  for (const auto & pose : path.poses) {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    EXPECT_NEAR(x * x + (y - 1.0) * (y - 1.0), 1.0, 1e-5);
  }
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, 1.0);
}

TEST_F(FootprintBTTest, ClearLatticeStopTurnBecomesContinuousCheckedCorner)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & p : std::vector<std::tuple<double, double, double>>{
      {0.0, 0.0, 0.0}, {.2, 0.0, 0.0}, {.4, 0.0, 0.0},
      {.4, 0.0, .35}, {.59, .069, .35}, {.78, .139, .35}, {.95, .20, .35}})
  {
    geometry_msgs::msg::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = std::get<0>(p);
    point.pose.position.y = std::get<1>(p);
    const double yaw = std::get<2>(p);
    point.pose.orientation.z = std::sin(yaw / 2);
    point.pose.orientation.w = std::cos(yaw / 2);
    path.poses.push_back(point);
  }
  board->set("path", path);
  geometry_msgs::msg::PoseStamped goal = path.poses.back();
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' goal='{goal}' planner_id='SE2Fallback' "
    "checked_path='{rounded}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto rounded = board->get<nav_msgs::msg::Path>("rounded");
  ASSERT_GT(rounded.poses.size(), path.poses.size());
  for (size_t i = 1; i < rounded.poses.size(); ++i) {
    const auto & a = rounded.poses[i - 1].pose;
    const auto & b = rounded.poses[i].pose;
    const double d = std::hypot(b.position.x - a.position.x,
      b.position.y - a.position.y);
    const double turn = std::abs(std::remainder(
      tf2::getYaw(b.orientation) - tf2::getYaw(a.orientation), 6.283185307179586));
    EXPECT_FALSE(d < .005 && turn > .02);
  }
  // This cell lies in the moving fillet's rear sweep while the original
  // stopped turn remains clear. Keep the original maneuver in that case.
  map.data[67 * 160 + 81] = 254;
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto fallback = board->get<nav_msgs::msg::Path>("rounded");
  EXPECT_EQ(fallback.poses.size(), path.poses.size());
}

TEST_F(FootprintBTTest, NewGoalClearsOldRecoveryErrorState)
{
  selectMotion("recovery_allowed");
  board->set<uint16_t>("compute_path_error_code", 205);
  board->set<uint16_t>("follow_path_error_code", 106);
  board->set("local_escape_required", true);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState/></BehaviorTree></root>", board);
  EXPECT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<uint16_t>("compute_path_error_code"), 0);
  EXPECT_EQ(board->get<uint16_t>("follow_path_error_code"), 0);
  EXPECT_FALSE(board->get<bool>("local_escape_required"));
}

TEST_F(FootprintBTTest, ReadinessWaitsForFirstCallbackInsteadOfAbortingGoal)
{
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<WaitForClearanceData/></BehaviorTree></root>", board);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // A previously unvisited recovery query uses the SAME warmed subscriptions.
  selectMotion("forward");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, ShortWestboundXYPathDoesNotRequestIdentityYawSpin)
{
  robot_transform.rotation.z = 1.0;
  robot_transform.rotation.w = 0.0;
  robot_transform.translation.x = robot_transform.translation.y = 0.0125;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {0.0, -0.10}) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    p.pose.orientation.w = 1.0;  // reproduce short Smac2D default yaw
    path.poses.push_back(p);
  }
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = -0.12;  // requested goal differs from grid-cell endpoint
  goal.pose.position.y = 0.0125;
  goal.pose.orientation.z = 1.0;
  board->set("path", path);
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' goal='{goal}' planner_id='GridBased' "
    "checked_path='{path}'/></BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto checked = board->get<nav_msgs::msg::Path>("path");
  EXPECT_NEAR(std::abs(checked.poses.front().pose.orientation.z), 1.0, 1e-6);
  EXPECT_NEAR(checked.poses.back().pose.orientation.z, 1.0, 1e-6);
  EXPECT_DOUBLE_EQ(checked.poses.back().pose.position.x, -0.12);
  EXPECT_NEAR(checked.poses.front().pose.position.x, 0.0125, 1e-8);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' goal='{goal}' planner_id='GridBased' "
    "prepared='true' checked_path='{path}'/></BehaviorTree></root>", board);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(board->get<nav_msgs::msg::Path>("path").poses.front().pose.position.x,
    checked.poses.front().pose.position.x);
}

TEST_F(FootprintBTTest, ReplayStationaryRobotCapture)
{
  const auto filename = std::getenv("TAI_CLEARANCE_CAPTURE");
  if (!filename) {GTEST_SKIP() << "Set TAI_CLEARANCE_CAPTURE for captured costmap regression";}
  std::ifstream input(filename);
  ASSERT_TRUE(input.good());
  const auto data = nlohmann::json::parse(input);
  auto load_map = [](const auto & j) {
      nav2_msgs::msg::Costmap msg;
      msg.header.frame_id = j["header"]["frame_id"].template get<std::string>();
      const auto & m = j["metadata"];
      msg.metadata.size_x = m["size_x"]; msg.metadata.size_y = m["size_y"];
      msg.metadata.resolution = m["resolution"];
      msg.metadata.origin.position.x = m["origin"]["position"]["x"];
      msg.metadata.origin.position.y = m["origin"]["position"]["y"];
      msg.metadata.origin.orientation.w = 1.0;
      msg.data = j["data"].template get<std::vector<uint8_t>>();
      return msg;
    };
  auto load_tf = [](const auto & j) {
      tf2::Transform result;
      result.setOrigin(tf2::Vector3(j["translation"]["x"].template get<double>(),
        j["translation"]["y"].template get<double>(), 0));
      result.setRotation(tf2::Quaternion(0, 0,
        j["rotation"]["z"].template get<double>(),
        j["rotation"]["w"].template get<double>()));
      return result;
    };
  map = load_map(data["global"]);
  local_map = load_map(data["local"]);
  split_frames = true;
  const auto odom_from_map = load_tf(data["map_to_local"]["transform"]);
  localization_transform = tf2::toMsg(odom_from_map.inverse());
  robot_transform = tf2::toMsg(odom_from_map * load_tf(data["robot"]["transform"]));
  const double advance = std::getenv("TAI_CLEARANCE_ADVANCE") ?
    std::stod(std::getenv("TAI_CLEARANCE_ADVANCE")) : 0.0;
  const double robot_yaw = tf2::getYaw(robot_transform.rotation);
  robot_transform.translation.x += advance * std::cos(robot_yaw);
  robot_transform.translation.y += advance * std::sin(robot_yaw);
  const auto body_from_odom = load_tf(data["footprint_to_base"]["transform"]);
  body.polygon.points.clear();
  for (const auto & p : data["footprint"]["polygon"]["points"]) {
    const auto q = body_from_odom * tf2::Vector3(p["x"].get<double>(), p["y"].get<double>(), 0);
    geometry_msgs::msg::Point32 point;
    point.x = q.x(); point.y = q.y();
    body.polygon.points.push_back(point);
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & probe : data["probes"]) {
    if (std::abs(probe["advance"].get<double>() - advance) > 1e-6 ||
      probe["planner"] != "SE2Fallback") {continue;}
    for (const auto & p : probe["path"]["poses"]) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p["pose"]["position"]["x"];
      pose.pose.position.y = p["pose"]["position"]["y"];
      pose.pose.orientation.z = p["pose"]["orientation"]["z"];
      pose.pose.orientation.w = p["pose"]["orientation"]["w"];
      path.poses.push_back(pose);
    }
  }
  if (data.contains("selected")) {
    for (const auto & p : data["selected"]["poses"]) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p["pose"]["position"]["x"];
      pose.pose.position.y = p["pose"]["position"]["y"];
      pose.pose.orientation.z = p["pose"]["orientation"]["z"];
      pose.pose.orientation.w = p["pose"]["orientation"]["w"];
      path.poses.push_back(pose);
    }
    tree = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
      "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  }
  if (std::getenv("TAI_CLEARANCE_MOTION")) {
    selectMotion(std::getenv("TAI_CLEARANCE_MOTION"));
    auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
    const auto & requested = data.at("requested_goal");
    goal.pose.position.x = requested[0];
    goal.pose.position.y = requested[1];
    const double a = requested[2];
    goal.pose.orientation.z = std::sin(a / 2);
    goal.pose.orientation.w = std::cos(a / 2);
    board->set("goal", goal);
  } else {
    ASSERT_FALSE(path.poses.empty());
  }
  if (std::getenv("TAI_CLEARANCE_MOTION") &&
    std::string(std::getenv("TAI_CLEARANCE_MOTION")) == "forward_route")
  {
    const auto current = load_tf(data["robot"]["transform"]);
    const double yaw = tf2::getYaw(current.getRotation());
    geometry_msgs::msg::PoseStamped candidate;
    candidate.header.frame_id = "map";
    candidate.pose.position.x = current.getOrigin().x() + advance * std::cos(yaw);
    candidate.pose.position.y = current.getOrigin().y() + advance * std::sin(yaw);
    candidate.pose.orientation = tf2::toMsg(current.getRotation());
    board->set("candidate", candidate);
    robot_transform.translation.x -= advance * std::cos(robot_yaw);
    robot_transform.translation.y -= advance * std::sin(robot_yaw);
    tree = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
      "<PathFootprintClear motion='forward_route' path='{path}' goal='{goal}' "
      "candidate_start='{candidate}' planner_id='SE2Fallback' checked_path='{joined}'/>"
      "</BehaviorTree></root>", board);
  }
  board->set("path", path);
  if (std::getenv("TAI_CLEARANCE_MOTION") &&
    std::string(std::getenv("TAI_CLEARANCE_MOTION")) == "replan_after_retreat")
  {
    const auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
    const auto stopped = robot_transform;
    // Synthetic odometry only, in isolated domain 231: reproduce one real
    // retreat from 15 cm ahead to the captured rear-blocked stopping pose.
    robot_transform.translation.x += 0.15 * std::cos(robot_yaw);
    robot_transform.translation.y += 0.15 * std::sin(robot_yaw);
    selectMotion("reverse_needed");
    board->set("goal", goal);
    connect();
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    robot_transform = stopped;
    rclcpp::sleep_for(std::chrono::milliseconds(220));
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(40));
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    selectMotion("replan_after_retreat");
    board->set("goal", goal);
  } else {
    connect();
  }
  const auto expected = std::getenv("TAI_CLEARANCE_EXPECT_FAILURE") ?
    BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  const auto status = tree.tickOnce();
  double angle = 0.0;
  (void)board->get("angle", angle);
  std::cout << "Replay hypothetical advance=" << advance << ", status=" << status <<
    ", turn=" << angle << " rad\n";
  EXPECT_EQ(status, expected);
}

TEST_F(FootprintBTTest, ForwardCandidateChecksPrefixWithoutRequiringImmediateGoalFacingTurn)
{
  selectMotion("forward_candidate");
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // Collision inside the current body must never be ignored by this search.
  map.data[80 * 160 + 80] = 254;
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, RunningTurnValidatesRemainingSweepWithoutRevisitingOldHeading)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped point;
  point.pose.orientation.w = 1.0;
  path.poses.push_back(point);
  point.pose.orientation.z = std::sin(1.3 / 2);
  point.pose.orientation.w = std::cos(1.3 / 2);
  path.poses.push_back(point);
  point.pose.position.x = .1;
  point.pose.position.y = .9;
  path.poses.push_back(point);
  board->set("path", path);
  robot_transform.rotation.z = std::sin(1.2 / 2);
  robot_transform.rotation.w = std::cos(1.2 / 2);
  // Newly observed obstacle intersects the old fork orientation, but neither
  // the current orientation, remaining 0.1 rad sweep, nor forward exit.
  map.data[80 * 160 + 108] = 254;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, PreparedTerminalTurnStillRequiresShortestFinalSweep)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped point;
  point.pose.orientation.z = std::sin(1.2 / 2);
  point.pose.orientation.w = std::cos(1.2 / 2);
  path.poses.push_back(point);
  point.pose.orientation.z = std::sin(-1.2 / 2);
  point.pose.orientation.w = std::cos(-1.2 / 2);
  path.poses.push_back(point);
  board->set("path", path);
  robot_transform.rotation = path.poses.front().pose.orientation;
  map.data[80 * 160 + 108] = 254;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' prepared='true'/></BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, TurnLimitSkipsFutilePathsUntilRealEscapeTranslation)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  auto limited = node->create_publisher<std_msgs::msg::Bool>(
    "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='route_control_available'/></BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  std_msgs::msg::Bool signal;
  signal.data = true;
  BT::NodeStatus status = BT::NodeStatus::SUCCESS;
  for (int i = 0; i < 20 && status == BT::NodeStatus::SUCCESS; ++i) {
    limited->publish(signal);
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(30));
    status = tree.tickOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  // An AMCL correction cannot reset a physical stationary turn budget.
  localization_transform.translation.x = .30;
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  robot_transform.translation.x = .06;
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, TurnLimitAuthorizesCheckedEscapeWithoutControllerError)
{
  selectMotion("recovery_allowed");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  auto limited = node->create_publisher<std_msgs::msg::Bool>(
    "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool signal;
  signal.data = true;
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  for (int i = 0; i < 20 && status == BT::NodeStatus::FAILURE; ++i) {
    limited->publish(signal);
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(30));
    status = tree.tickOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  // A geometric guard signal cannot override a controller/TF fault or
  // authorize moving away from an already-reached terminal XY.
  board->set<uint16_t>("controller", 107);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  board->set<uint16_t>("controller", 0);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = .05;
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, LimitedTurnAdvancesBeforeAnOtherwiseUsableStationaryTurn)
{
  selectMotion("forward");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -1.0;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  auto limited = node->create_publisher<std_msgs::msg::Bool>(
    "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool signal;
  signal.data = true;
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  for (int i = 0; i < 20 && status == BT::NodeStatus::FAILURE; ++i) {
    limited->publish(signal);
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(30));
    status = tree.tickOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(board->get<double>("distance"), .15);
}

TEST_F(FootprintBTTest, LimitedTurnRetreatsWhenCheckedAdvanceIsBlocked)
{
  selectMotion("reverse_needed");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -1.0;
  board->set("goal", goal);
  map.data[80 * 160 + 117] = 254;  // x=.9375 blocks a .15 m advance
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  auto limited = node->create_publisher<std_msgs::msg::Bool>(
    "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool signal;
  signal.data = true;
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  for (int i = 0; i < 20 && status == BT::NodeStatus::FAILURE; ++i) {
    limited->publish(signal);
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(30));
    status = tree.tickOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -.06;
  // Expensive reverse geometry is cached for at most 200 ms.
  rclcpp::sleep_for(std::chrono::milliseconds(210));
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);  // hand off at new pocket
}

TEST_F(FootprintBTTest, NewGoalCanReachControllerAfterPreviousTurnLimit)
{
  selectMotion("route_control_available");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto limited = node->create_publisher<std_msgs::msg::Bool>(
    "/navigation/stationary_turn_limited", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool signal;
  signal.data = true;
  BT::NodeStatus status = BT::NodeStatus::SUCCESS;
  for (int i = 0; i < 20 && status == BT::NodeStatus::SUCCESS; ++i) {
    limited->publish(signal);
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(30));
    status = tree.tickOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.y = .5;
  board->set("goal", goal);
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, HeadingAwarePathsFinishAtRequestedPoseInsteadOfQuantizedEndpoint)
{
  auto raw = board->get<nav_msgs::msg::Path>("path");
  raw.poses.back().pose.position.x = 0.88;
  raw.poses.back().pose.orientation.z = std::sin(0.25);
  raw.poses.back().pose.orientation.w = std::cos(0.25);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 0.90;
  goal.pose.orientation.w = 1.0;
  board->set("goal", goal);
  for (const std::string planner : {"SE2Arc", "SE2Fallback"}) {
    board->set("path", raw);
    tree = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
      "<PathFootprintClear path='{path}' goal='{goal}' planner_id='" + planner +
      "' checked_path='{checked}'/></BehaviorTree></root>", board);
    connect();
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    const auto checked = board->get<nav_msgs::msg::Path>("checked");
    EXPECT_DOUBLE_EQ(checked.poses.back().pose.position.x, .90);
    EXPECT_NEAR(tf2::getYaw(checked.poses.back().pose.orientation), 0.0, 1e-9);
  }
}

TEST_F(FootprintBTTest, ForwardRouteIncludesActualDepartureAndExactGoal)
{
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = "map";
  start.pose.orientation.w = goal.pose.orientation.w = 1.0;
  start.pose.position.x = 0.15;
  goal.pose.position.x = 0.90;
  nav_msgs::msg::Path raw;
  raw.header.frame_id = "map";
  raw.poses = {start, goal};
  board->set("path", raw);
  board->set("candidate", start);
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_route' path='{path}' goal='{goal}' "
    "candidate_start='{candidate}' planner_id='SE2Fallback' checked_path='{joined}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto joined = board->get<nav_msgs::msg::Path>("joined");
  EXPECT_NEAR(joined.poses.front().pose.position.x, 0, 1e-9);
  EXPECT_NEAR(joined.poses.back().pose.position.x, 0.90, 1e-9);
  EXPECT_GT(joined.poses.size(), raw.poses.size());
  goal.pose.position.x = 1.15;
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);  // Wrong destination.
  goal.pose.position.x = 0.90;
  board->set("goal", goal);
  map.data[80 * 160 + 72] = 254;  // Rear of current body, absent at future start.
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, ForwardPrefixBeyondOneMeterStillChecksFreshLocalObstacles)
{
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = "map";
  start.pose.orientation.w = goal.pose.orientation.w = 1.0;
  start.pose.position.x = 1.05;
  goal.pose.position.x = 1.15;
  nav_msgs::msg::Path raw;
  raw.header.frame_id = "map";
  raw.poses = {start, goal};
  board->set("path", raw); board->set("candidate", start); board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_route' path='{path}' goal='{goal}' "
    "candidate_start='{candidate}' planner_id='SE2Fallback' checked_path='{joined}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  local_map.data[80 * 160 + 153] = 254;  // x=1.8375, reached only after centre x>1 m.
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, ControllerFaultNeverAuthorizesLocalMotionEvenWithStartOccupied)
{
  selectMotion("recovery_allowed");
  board->set<uint16_t>("planner", 205);
  board->set("blocked", true);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  for (const uint16_t fault : {100, 101, 102, 103, 107}) {
    board->set<uint16_t>("controller", fault);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
}

TEST_F(FootprintBTTest, CheckedEscapeAllowedOutsideFiveCentimeterGoalTolerance)
{
  selectMotion("recovery_allowed");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = .08;
  board->set("goal", goal);
  board->set("blocked", true);
  board->set<uint16_t>("controller", 104);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  goal.pose.position.x = .04;
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, CloseGoalBehindAllowsOnlyCheckedLocalEscape)
{
  selectMotion("recovery_allowed");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -.18;
  board->set("goal", goal);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  board->set<uint16_t>("planner", 208);  // No valid path, still safe to check rear.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  board->set<uint16_t>("planner", 207);  // Bounded search timeout, rear still checked.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  board->set<uint16_t>("planner", 202);  // TF fault does not authorize motion.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  board->set<uint16_t>("planner", 0);
  goal.pose.position.x = .18;
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, CloseBehindGoalRetreatsBeforeChoosingTurn)
{
  selectMotion("recovery_allowed");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -.18;
  board->set("goal", goal);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  selectMotion("turn");
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  selectMotion("reverse_needed");
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -.26;
  publish();
  selectMotion("escape_available");
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, NearbyBehindGoalRejectsForwardDepartureProposal)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = -.18;
  goal.pose.orientation.w = 1;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_candidate' goal='{goal}' "
    "candidate_distance='0.15' forward_start='{start}'/>"
    "</BehaviorTree></root>", board);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

class SyntheticForwardProposal : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static inline bool blocked = false, planning = false, detours = false, block_direct = false;
  static inline bool direct_lower_bound = false;
  static inline int cancels = 0;
  static inline std::vector<double> distances;
  static inline std::vector<std::string> requests;
  static inline double goal_yaw = 0.0;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("distance"), BT::InputPort<std::string>("request", "", "Planner"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<std::string>("planner")};
  }
  BT::NodeStatus onStart() override
  {
    distances.push_back(getInput<double>("distance").value());
    requests.push_back(getInput<std::string>("request").value());
    return planning ? BT::NodeStatus::RUNNING : finish();
  }
  BT::NodeStatus onRunning() override {return planning ? BT::NodeStatus::RUNNING : finish();}
  void onHalted() override {++cancels;}
private:
  BT::NodeStatus finish()
  {
    if (blocked || (block_direct && getInput<std::string>("request").value() == "Direct")) {
      return BT::NodeStatus::FAILURE;
    }
    const double d = getInput<double>("distance").value();
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped p;
    p.pose.orientation.w = 1;
    path.poses.push_back(p);
    p.pose.position.x = std::abs(d - 0.45) < 1e-6 ? 1.0 :
      (std::abs(d - 0.15) < 1e-6 ? 1.2 : 2.0 + d);
    path.poses.push_back(p);
    const auto request = getInput<std::string>("request").value();
    if (!request.empty()) {
      // Deliberately offer the longest valid route first. The lattice offers
      // the shortest route last; first-success planning must not accept arcs.
      const double extra = request == "Direct" ? (direct_lower_bound ? 0.0 : 6.0) :
        request == "DirectArc" ? 5.0 :
        request == "GridShortest" ? 4.5 :
        request == "GridBased" ? 4.0 : request == "SE2Arc" ? 2.0 : 0.0;
      path.poses.back().pose.position.x += extra;
    }
    if (detours && d == 0.0) {
      auto middle = path.poses.back();
      middle.pose.position.x += 4.0;
      middle.pose.position.y = 2.0;
      path.poses.insert(path.poses.end() - 1, middle);
    }
    path.poses.back().pose.orientation.z = std::sin(goal_yaw / 2.0);
    path.poses.back().pose.orientation.w = std::cos(goal_yaw / 2.0);
    setOutput("path", path);
    setOutput("planner", request.empty() ? std::string("SE2Fallback") : request);
    return BT::NodeStatus::SUCCESS;
  }
};

class CheckRankedRoute : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static inline bool reject_best = false;
  static inline std::vector<double> checked;
  static BT::PortsList providedPorts() {return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override
  {
    const double length = getInput<nav_msgs::msg::Path>("path").value().poses.back().pose.position.x;
    checked.push_back(length);
    return reject_best && length == 1.0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class ExecuteRankedRoute : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static inline bool fail_first = false, fail_all = false, running = false;
  static inline uint16_t error = 104;
  static inline std::vector<double> executed;
  static BT::PortsList providedPorts() {return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus onStart() override
  {
    executed.push_back(getInput<nav_msgs::msg::Path>("path").value().poses.back().pose.position.x);
    if (fail_all || (fail_first && executed.size() == 1)) {
      config().blackboard->set<uint16_t>("controller", error);
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set<uint16_t>("controller", 0);
    return running ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {}
};

class ForwardSearchBTTest : public FootprintBTTest
{
protected:
  void SetUp() override
  {
    FootprintBTTest::SetUp();
    SyntheticForwardProposal::blocked = SyntheticForwardProposal::planning =
      SyntheticForwardProposal::detours = false;
    SyntheticForwardProposal::block_direct = false;
    SyntheticForwardProposal::direct_lower_bound = false;
    SyntheticForwardProposal::cancels = 0; SyntheticForwardProposal::distances.clear();
    SyntheticForwardProposal::requests.clear();
    SyntheticForwardProposal::goal_yaw = 0.0;
    CheckRankedRoute::reject_best = false; CheckRankedRoute::checked.clear();
    ExecuteRankedRoute::fail_first = ExecuteRankedRoute::fail_all =
      ExecuteRankedRoute::running = false;
    ExecuteRankedRoute::error = 104; ExecuteRankedRoute::executed.clear();
    board->set<uint16_t>("controller", 0);
    board->set("budget", 60.0);
    factory.registerNodeType<SyntheticForwardProposal>("SyntheticForwardProposal");
    factory.registerNodeType<CheckRankedRoute>("CheckRankedRoute");
    factory.registerNodeType<ExecuteRankedRoute>("ExecuteRankedRoute");
    tree = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
      "<ForwardRouteSearch candidate_distance='{distance}' candidate_path='{candidate}' "
      "candidate_planner='{candidate_planner}' controller_error='{controller}' "
      "path='{selected}' planner_id='{selected_planner}' search_timeout='{budget}'>"
      "<SyntheticForwardProposal distance='{distance}' path='{candidate}' planner='{candidate_planner}'/>"
      "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
      "</ForwardRouteSearch></BehaviorTree></root>", board);
  }
  BT::NodeStatus complete()
  {
    for (int i = 0; i < 40; ++i) {
      const auto result = tree.tickOnce();
      if (result != BT::NodeStatus::RUNNING) {return result;}
    }
    return BT::NodeStatus::RUNNING;
  }
};

TEST_F(ForwardSearchBTTest, CurrentPoseComparesAllPlannersAndChoosesShortestLastCandidate)
{
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' candidate_distance='{distance}' "
    "candidate_planner_request='{request}' candidate_path='{candidate}' "
    "candidate_planner='{candidate_planner}' controller_error='{controller}' "
    "path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(SyntheticForwardProposal::requests,
    (std::vector<std::string>{"Direct", "DirectArc", "GridShortest", "GridBased", "SE2Arc",
      "SE2Fallback"}));
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 2.0);
  EXPECT_EQ(board->get<std::string>("selected_planner"), "SE2Fallback");
}

TEST_F(ForwardSearchBTTest, ValidDirectLowerBoundSkipsExpensivePlannerTrials)
{
  SyntheticForwardProposal::direct_lower_bound = true;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2.0;
  goal.pose.orientation.w = 1.0;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' explore_forward_detours='true' "
    "goal='{goal}' candidate_distance='{distance}' candidate_planner_request='{request}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(SyntheticForwardProposal::requests,
    (std::vector<std::string>{"Direct"}));
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 2.0);
  EXPECT_EQ(board->get<std::string>("selected_planner"), "Direct");
}

TEST_F(ForwardSearchBTTest, SuccessfulRecoveryRouteFinishesWithoutAnotherFollowPath)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2;
  goal.pose.orientation.w = 1;
  board->set("goal", goal);
  factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_recovery_node_bt_node.so");
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RecoveryNode number_of_retries='1'>"
    "<PathFootprintClear motion='goal_completed' goal='{goal}'/>"
    "<RankedRouteSearch forward_search='false' goal='{goal}' candidate_distance='{distance}' "
    "candidate_planner_request='{request}' candidate_path='{candidate}' "
    "candidate_planner='{candidate_planner}' controller_error='{controller}' "
    "path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></RecoveryNode></BehaviorTree></root>", board);
  ASSERT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  goal.pose.position.y = .01;
  board->set("goal", goal);
  auto completed = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='goal_completed' goal='{goal}'/></BehaviorTree></root>", board);
  EXPECT_EQ(completed.tickOnce(), BT::NodeStatus::FAILURE);
  goal.pose.position.y = 0;
  board->set("goal", goal);
  EXPECT_EQ(completed.tickOnce(), BT::NodeStatus::SUCCESS);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState reset_motion_history='true'/></BehaviorTree></root>", board);
  EXPECT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(completed.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ForwardSearchBTTest, LongCurrentPoseDetourIsComparedWithForwardDeparturesBeforeMotion)
{
  SyntheticForwardProposal::detours = true;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' explore_forward_detours='true' explore_goal_approaches='true' "
    "candidate_distance='{distance}' candidate_planner_request='{request}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  // The GitHub baseline searches six current routes and 24 forward starts.
  // Aligned goal entries are reserved for the case with no current route.
  for (int i = 0; i < 30; ++i) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
  }
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(SyntheticForwardProposal::distances.size(), 30u);
  EXPECT_DOUBLE_EQ(SyntheticForwardProposal::distances[0], 0.0);
  EXPECT_DOUBLE_EQ(SyntheticForwardProposal::distances[6], .15);
  EXPECT_DOUBLE_EQ(SyntheticForwardProposal::distances[29], 1.20);
  EXPECT_DOUBLE_EQ(SyntheticForwardProposal::distances.back(), 1.20);
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 1.0);
}

TEST_F(ForwardSearchBTTest, NearGoalDefersLongRouteUntilAfterLocalEscape)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = .10;
  goal.pose.orientation.w = 1;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' goal='{goal}' max_near_goal_length='1.5' "
    "candidate_distance='{distance}' candidate_planner_request='{request}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' goal='{goal}' max_near_goal_length='0.0' "
    "candidate_distance='{distance}' candidate_planner_request='{request}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 2.0);
}

TEST_F(ForwardSearchBTTest, ShortCurrentPoseRoutesDoNotTriggerExtraDepartureSearch)
{
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' explore_forward_detours='true' "
    "candidate_distance='{distance}' candidate_planner_request='{request}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(SyntheticForwardProposal::distances.size(), 6u);
}

TEST_F(ForwardSearchBTTest, ClearRouteWithLargeTerminalPivotDoesNotAddApproachDetours)
{
  SyntheticForwardProposal::goal_yaw = 1.2;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' explore_forward_detours='true' "
    "explore_goal_approaches='true' candidate_distance='{distance}' "
    "candidate_planner_request='{request}' candidate_departure_mode='{mode}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(SyntheticForwardProposal::distances.size(), 6u);
  for (const double d : SyntheticForwardProposal::distances) {EXPECT_DOUBLE_EQ(d, 0.0);}
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
}

TEST_F(ForwardSearchBTTest, BlockedDirectLineDoesNotAddAlignedApproachDetour)
{
  SyntheticForwardProposal::goal_yaw = 1.2;
  SyntheticForwardProposal::block_direct = true;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' explore_forward_detours='true' "
    "explore_goal_approaches='true' candidate_distance='{distance}' "
    "candidate_planner_request='{request}' candidate_path='{candidate}' "
    "candidate_planner='{candidate_planner}' controller_error='{controller}' "
    "path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(SyntheticForwardProposal::distances.size(), 6u);
}

TEST_F(ForwardSearchBTTest, PrimaryFailureRequestsFreshPlanningBeforeLongerStoredRoutes)
{
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' refresh_on_failure='true' "
    "candidate_distance='{distance}' candidate_planner_request='{request}' "
    "candidate_path='{candidate}' candidate_planner='{candidate_planner}' "
    "controller_error='{controller}' path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  ExecuteRankedRoute::fail_first = true;
  EXPECT_EQ(complete(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 2.0);
}

TEST_F(ForwardSearchBTTest, ForwardSearchComparesEveryPlannerAtEveryDeparture)
{
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ForwardRouteSearch compare_planners='true' candidate_distance='{distance}' "
    "candidate_planner_request='{request}' candidate_path='{candidate}' "
    "candidate_planner='{candidate_planner}' controller_error='{controller}' "
    "path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</ForwardRouteSearch></BehaviorTree></root>", board);
  for (int i = 0; i < 24; ++i) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
  }
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(SyntheticForwardProposal::requests.size(), 24u);
  for (size_t i = 0; i < 24; ++i) {
    EXPECT_NEAR(SyntheticForwardProposal::distances[i], .15 * (i / 3 + 1), 1e-9);
    EXPECT_EQ(SyntheticForwardProposal::requests[i],
      (std::vector<std::string>{"GridBased", "SE2Arc", "SE2Fallback"})[i % 3]);
  }
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 1.0);
}

TEST_F(ForwardSearchBTTest, ChangedGoalCancelsExecutionAndRanksFreshRoutes)
{
  geometry_msgs::msg::PoseStamped initial_goal;
  initial_goal.header.frame_id = "map";
  initial_goal.pose.orientation.w = 1.0;
  board->set("goal", initial_goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RankedRouteSearch forward_search='false' goal='{goal}' candidate_distance='{distance}' "
    "candidate_planner_request='{request}' candidate_path='{candidate}' "
    "candidate_planner='{candidate_planner}' controller_error='{controller}' "
    "path='{selected}' planner_id='{selected_planner}'>"
    "<SyntheticForwardProposal distance='{distance}' request='{request}' "
    "path='{candidate}' planner='{candidate_planner}'/>"
    "<CheckRankedRoute path='{selected}'/><ExecuteRankedRoute path='{selected}'/>"
    "</RankedRouteSearch></BehaviorTree></root>", board);
  ExecuteRankedRoute::running = true;
  for (int i = 0; i < 8; ++i) {EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);}
  ASSERT_EQ(SyntheticForwardProposal::requests.size(), 6u);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.y += 1.0;
  board->set("goal", goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(SyntheticForwardProposal::requests.size(), 7u);
  for (int i = 0; i < 8; ++i) {EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);}
  EXPECT_EQ(SyntheticForwardProposal::requests.size(), 12u);
  EXPECT_EQ(ExecuteRankedRoute::executed.size(), 2u);
}

TEST_F(ForwardSearchBTTest, SearchesBeforeMotionAndExecutesBestRankedCompleteRoute)
{
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
  }
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(SyntheticForwardProposal::distances.size(), 8u);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_NEAR(SyntheticForwardProposal::distances[i], 0.15 * (i + 1), 1e-9);
  }
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 1.0);
}

TEST_F(ForwardSearchBTTest, GeometricFailureTriesNextRankedRouteBeforeReverse)
{
  ExecuteRankedRoute::fail_first = true;
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 2u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 1.0);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[1], 1.2);
}

TEST_F(ForwardSearchBTTest, RevalidatesAndSkipsNewlyBlockedBestRoute)
{
  CheckRankedRoute::reject_best = true;
  EXPECT_EQ(complete(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(ExecuteRankedRoute::executed.size(), 1u);
  EXPECT_DOUBLE_EQ(ExecuteRankedRoute::executed[0], 1.2);
}

TEST_F(ForwardSearchBTTest, TfFaultDoesNotExecuteOtherRoutes)
{
  ExecuteRankedRoute::fail_first = true; ExecuteRankedRoute::error = 102;
  EXPECT_EQ(complete(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ExecuteRankedRoute::executed.size(), 1u);
}

TEST_F(ForwardSearchBTTest, ExhaustsFiniteCandidatesWithoutMotionWhenAllBlocked)
{
  SyntheticForwardProposal::blocked = true;
  EXPECT_EQ(complete(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(SyntheticForwardProposal::distances.size(), 8u);
  EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
}

TEST_F(ForwardSearchBTTest, ExhaustsExecutionAlternativesWithoutStationaryRetryLoop)
{
  ExecuteRankedRoute::fail_all = true; ExecuteRankedRoute::error = 106;
  EXPECT_EQ(complete(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ExecuteRankedRoute::executed.size(), 8u);
}

TEST_F(ForwardSearchBTTest, HaltCancelsPlanningAndResetsChoicesForNewGoal)
{
  SyntheticForwardProposal::planning = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(SyntheticForwardProposal::cancels, 1);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(SyntheticForwardProposal::distances.size(), 2u);
  EXPECT_DOUBLE_EQ(SyntheticForwardProposal::distances[0], 0.15);
  EXPECT_DOUBLE_EQ(SyntheticForwardProposal::distances[1], 0.15);
  EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
}

TEST_F(ForwardSearchBTTest, RunningExecutionRetainsChosenPathWithoutRepeatedPlanning)
{
  ExecuteRankedRoute::running = true;
  EXPECT_EQ(complete(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(SyntheticForwardProposal::distances.size(), 8u);
  EXPECT_EQ(ExecuteRankedRoute::executed.size(), 1u);
}

TEST_F(ForwardSearchBTTest, PlanningDeadlineCancelsHungPlannerWithoutMotion)
{
  SyntheticForwardProposal::planning = true;
  board->set("budget", 0.01);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  rclcpp::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(SyntheticForwardProposal::cancels, 1);
  EXPECT_TRUE(ExecuteRankedRoute::executed.empty());
}

TEST_F(ForwardSearchBTTest, TerminalSuccessAndFailureClearChoicesForNextRecoveryExecution)
{
  SyntheticForwardProposal::blocked = true;
  ASSERT_EQ(complete(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(SyntheticForwardProposal::distances.size(), 8u);
  SyntheticForwardProposal::blocked = false;
  ASSERT_EQ(complete(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(SyntheticForwardProposal::distances.size(), 16u);
  ASSERT_EQ(complete(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(SyntheticForwardProposal::distances.size(), 24u);
  EXPECT_EQ(ExecuteRankedRoute::executed.size(), 2u);
}

class NoMotionAction : public BT::SyncActionNode
{
public:
  NoMotionAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}
  BT::NodeStatus tick() override {return BT::NodeStatus::SUCCESS;}
};

class ReplanProbe : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {};}
  static inline int attempts = 0;
  BT::NodeStatus tick() override
  {
    return ++attempts == 1 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

TEST_F(FootprintBTTest, RearBlockedAfterContinuousRetreatReplansInsteadOfAborting)
{
  // A narrow forward dead end does not permit the full goal-facing turn.
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {map.data[iy * 160 + 114] = 254;}
  selectMotion("reverse_needed");
  connect();  // Match the real tree's WaitForClearanceData before planning.
  factory.registerNodeType<RunningBackup>("RunningBackup");
  factory.registerNodeType<ReplanProbe>("ReplanProbe");
  factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_recovery_node_bt_node.so");
  RunningBackup::starts = RunningBackup::stops = ReplanProbe::attempts = 0;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<RecoveryNode number_of_retries='2'><ReplanProbe/><Fallback>"
    "<ReactiveSequence><PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "<RunningBackup/></ReactiveSequence>"
    "<PathFootprintClear motion='escape_available' goal='{goal}'/>"
    "<Sequence><PathFootprintClear motion='replan_after_retreat' goal='{goal}'/>"
    "<ResetEscapeState invalidate_path='true'/></Sequence>"
    "</Fallback></RecoveryNode></BehaviorTree></root>", board);
  for (int i = 0; i < 20; ++i) {EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);}
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
  EXPECT_EQ(ReplanProbe::attempts, 1);
  robot_transform.translation.x = -0.25;
  // Current body fits, but another 16 cm of reverse does not.
  map.data[80 * 160 + 52] = 254;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 1);
  EXPECT_EQ(ReplanProbe::attempts, 2);
  EXPECT_TRUE(board->get<nav_msgs::msg::Path>("path").poses.empty());
  selectMotion("replan_after_retreat");
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);  // Consumed once.
}

TEST_F(FootprintBTTest, RetreatReplanRequiresRealReverseAndRejectsFaults)
{
  // A geometric BackUp collision may request planning, never blind motion.
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {map.data[iy * 160 + 114] = 254;}
  selectMotion("reverse_needed");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  selectMotion("replan_after_retreat");
  board->set<uint16_t>("motion_error", 714);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);  // No displacement.
  selectMotion("reverse_needed");
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -0.20;
  publish();
  selectMotion("replan_after_retreat");
  for (const uint16_t fault : {710, 711, 712, 713}) {
    board->set<uint16_t>("motion_error", fault);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
  board->set<uint16_t>("motion_error", 714);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, ForwardTravelLocalizationJumpAndNewGoalDoNotGrantRetreatReplan)
{
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {map.data[iy * 160 + 114] = 254;}
  split_frames = true;
  local_map = map;
  local_map.header.frame_id = "odom";
  selectMotion("reverse_needed");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  localization_transform.translation.x = -0.30;
  publish();
  selectMotion("replan_after_retreat");
  board->set<uint16_t>("motion_error", 714);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  localization_transform.translation.x = 0;
  selectMotion("reverse_needed");
  publish();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = 0.10;
  publish();
  selectMotion("replan_after_retreat");
  board->set<uint16_t>("motion_error", 714);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  robot_transform.translation.x = 0;
  selectMotion("reverse_needed");
  publish();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -0.20;
  publish();
  selectMotion("replan_after_retreat");
  board->set<uint16_t>("motion_error", 714);
  auto new_goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  new_goal.pose.position.y = 0.20;
  board->set("goal", new_goal);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, StaleDataAndNewExecutionCannotReuseRetreatReplan)
{
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {map.data[iy * 160 + 114] = 254;}
  selectMotion("reverse_needed");
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -0.20;
  publish();
  selectMotion("replan_after_retreat");
  board->set<uint16_t>("motion_error", 714);
  rclcpp::sleep_for(std::chrono::milliseconds(1100));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState reset_motion_history='true'/></BehaviorTree></root>", board);
  EXPECT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  publish();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, BackupIsContinuousAndHaltedWhenForwardExitAppears)
{
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  for (unsigned int iy = 68; iy < 92; ++iy) {
    map.data[iy * 160 + 114] = 254;
                                                                            }
  selectMotion("reverse_needed");
  factory.registerNodeType<RunningBackup>("RunningBackup");
  RunningBackup::starts = RunningBackup::stops = 0;
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><Fallback>"
    "<ReactiveSequence><PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "<RunningBackup/></ReactiveSequence>"
    "<PathFootprintClear motion='escape_available' goal='{goal}'/>"
    "</Fallback></BehaviorTree></root>", board);
  connect();
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
                                                                                    }
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
  for (unsigned int iy = 68; iy < 92; ++iy) {
    map.data[iy * 160 + 114] = 0;
                                                                          }
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish();
  rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 1);
}

class CheckedMockSpin : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static inline int starts = 0;
  BT::NodeStatus tick() override {++starts; return BT::NodeStatus::SUCCESS;}
};

TEST_F(FootprintBTTest, RealRetreatSubtreeHaltsBackupOnlyWhenFullExitFits)
{
  for (const auto & lib : {
    "compute_path_to_pose_action", "follow_path_action", "wait_action",
    "back_up_action", "drive_on_heading", "spin_action", "pipeline_sequence",
    "rate_controller", "recovery_node", "goal_updated_condition",
    "globally_updated_goal_condition"})
  {
    factory.registerFromPlugin(std::string("/opt/ros/jazzy/lib/libnav2_") +
      lib + "_bt_node.so");
  }
  for (const auto & name : {"BackUp", "Wait", "Spin"}) {
    const auto manifest = factory.manifests().at(name);
    factory.unregisterBuilder(name);
    factory.registerBuilder(manifest,
      [name = std::string(name)](const std::string & n, const BT::NodeConfig & config)
        -> std::unique_ptr<BT::TreeNode> {
        if (name == "BackUp") {return std::make_unique<RunningBackup>(n, config);}
        if (name == "Spin") {return std::make_unique<CheckedMockSpin>(n, config);}
        return std::make_unique<NoMotionAction>(n, config);
      });
  }
  factory.registerBehaviorTreeFromFile(REAL_ESCAPE_TREE);
  selectMotion("reverse_needed");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0.0; goal.pose.position.y = 1.4;
  board->set("goal", goal);
  for (unsigned int ix = 0; ix < 160; ++ix) {
    map.data[67 * 160 + ix] = 254;
    map.data[92 * 160 + ix] = 254;
  }
  connect();
  // Exercise the real autoremap call, not a hand-written approximation.
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='TestRetreatCall'>"
    "<SubTree ID='CheckedRetreatAndTurn' _autoremap='true'/>"
    "</BehaviorTree></root>", board);
  RunningBackup::starts = RunningBackup::stops = CheckedMockSpin::starts = 0;
  for (int i = 0; i < 10; ++i) {EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);}
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(CheckedMockSpin::starts, 0);
  // A small angled pocket still blocks the full goal-facing turn: retain
  // the SAME BackUp action instead of returning to repeated partial spins.
  map.data.assign(map.data.size(), 0);
  map.data[109 * 160 + 89] = 254;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 0);
  EXPECT_EQ(CheckedMockSpin::starts, 0);
  // Only a completely clear sweep and departure corridor allows handoff.
  map.data[109 * 160 + 89] = 0;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(RunningBackup::stops, 1);
  EXPECT_EQ(CheckedMockSpin::starts, 1);
  EXPECT_NEAR(board->get<double>("escape_turn_angle"), M_PI / 2, 1e-5);
  EXPECT_TRUE(board->get<nav_msgs::msg::Path>("path").poses.empty());
}

TEST_F(FootprintBTTest, RealEscapeTreeLoadsWithInstalledNav2PortContracts)
{
  for (const auto & lib : {
    "compute_path_to_pose_action", "follow_path_action", "wait_action",
    "back_up_action", "drive_on_heading", "spin_action", "pipeline_sequence",
    "rate_controller", "recovery_node",
    "goal_updated_condition", "globally_updated_goal_condition"})
  {
    factory.registerFromPlugin(std::string("/opt/ros/jazzy/lib/libnav2_") +
      lib + "_bt_node.so");
  }
  // Preserve the real Nav2 port declarations, but replace action builders
  // so tree construction never contacts hardware or requires action servers.
  for (const auto & name : {"ComputePathToPose", "FollowPath", "Wait", "BackUp", "DriveOnHeading",
      "Spin"})
  {
    const auto manifest = factory.manifests().at(name);
    factory.unregisterBuilder(name);
    factory.registerBuilder(manifest,
      [](const std::string & n, const BT::NodeConfig & config) {
        return std::make_unique<NoMotionAction>(n, config);
      });
  }
  board->set("goal", geometry_msgs::msg::PoseStamped{});
  EXPECT_NO_THROW(factory.createTreeFromFile(REAL_ESCAPE_TREE, board));
}

TEST_F(FootprintBTTest, NoValidPathAllowsLocalRecoveryOnlyWithGeometricEvidence)
{
  selectMotion("recovery_allowed");
  board->set<uint16_t>("planner", 208);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  board->set("blocked", true);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  for (const uint16_t fault : {200, 202, 203, 204, 206, 207}) {
    board->set<uint16_t>("planner", fault);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
  board->set<uint16_t>("planner", 208);
  board->set<uint16_t>("controller", 107);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, RetreatStaysContinuousAcrossRepeatedThirtyCentimeterIntervals)
{
  // This aisle permits 15-degree adjustments, but the complete exit turn
  // only fits in the open space behind x=-1.0. Distance alone cannot release
  // retreat commitment and re-enable those ineffective partial turns.
  map.metadata.size_x = 240;
  map.metadata.origin.position.x = -3.0;
  map.data.assign(240 * 160, 0);
  for (unsigned int ix = 80; ix < 240; ++ix) {
    map.data[54 * 240 + ix] = 254;
    map.data[105 * 240 + ix] = 254;
  }
  selectMotion("turn");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -1.8;
  goal.pose.position.y = 1.0;
  board->set("goal", goal);
  connect();
  auto begin = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn_begin' goal='{goal}' candidate_angle='{angle}'/>"
    "</BehaviorTree></root>", board);
  auto reset = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<ResetEscapeState invalidate_path='true' turn_completed='true'/>"
    "</BehaviorTree></root>", board);
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    ASSERT_LT(std::abs(board->get<double>("angle")), 0.7);
    ASSERT_EQ(begin.tickOnce(), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(reset.tickOnce(), BT::NodeStatus::SUCCESS);
  }
  factory.registerNodeType<RunningBackup>("RunningBackup");
  RunningBackup::starts = RunningBackup::stops = 0;
  auto backup = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'><Fallback>"
    "<ReactiveSequence><PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "<RunningBackup/></ReactiveSequence>"
    "<PathFootprintClear motion='escape_available' goal='{goal}'/>"
    "</Fallback></BehaviorTree></root>", board);
  for (double rear_travel : {0.0, .31, .62, .93, 1.24}) {
    robot_transform.translation.x = -rear_travel;
    rclcpp::sleep_for(std::chrono::milliseconds(220));
    publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
    ASSERT_EQ(backup.tickOnce(), BT::NodeStatus::RUNNING)
      << "Still in the aisle after " << rear_travel << " m; keep the same BackUp active";
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(RunningBackup::starts, 1);
    EXPECT_EQ(RunningBackup::stops, 0);
  }
  robot_transform.translation.x = -1.90;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  ASSERT_EQ(backup.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(RunningBackup::starts, 1);
  EXPECT_EQ(RunningBackup::stops, 1);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(board->get<double>("angle"), std::atan2(1.0, .10), 1e-5);
}

TEST_F(FootprintBTTest, CommittedRetreatRequiresLongerExitAndRechecksBeforeTurn)
{
  selectMotion("turn");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = 0;
  goal.pose.position.y = 1.5;
  board->set("goal", goal);
  // Outside the rotation and former 30 cm departure check, but inside
  // the longer departure corridor. A small open pocket is not an exit.
  map.data[134 * 160 + 80] = 254;
  connect();
  auto collision = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn_collision' goal='{goal}' motion_error='703'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(collision.tickOnce(), BT::NodeStatus::SUCCESS);
  auto reverse = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='reverse_needed' goal='{goal}'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(reverse.tickOnce(), BT::NodeStatus::SUCCESS);
  robot_transform.translation.x = -.20;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(reverse.tickOnce(), BT::NodeStatus::SUCCESS);
  board->set("angle", std::atan2(1.5, .20));
  auto begin = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='turn_begin' goal='{goal}' candidate_angle='{angle}'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(begin.tickOnce(), BT::NodeStatus::FAILURE);
  map.data[134 * 160 + 80] = 0;
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(reverse.tickOnce(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(begin.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(FootprintBTTest, AlignedRearGoalWithinOneMeterUsesCheckedBackwardRoute)
{
  selectMotion("direct");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -1.0;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}' path_planner='{chosen}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<std::string>("chosen"), "DirectReverse");
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GT(path.poses.size(), 30u);
  for (const auto & point : path.poses) {EXPECT_NEAR(tf2::getYaw(point.pose.orientation), 0, 1e-8);}
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, -1.0);
  robot_transform.translation.x = -.96;
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  auto prepared = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear path='{path}' goal='{goal}' planner_id='DirectReverse' prepared='true'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(prepared.tickOnce(), BT::NodeStatus::SUCCESS);
  // A live obstacle along the full rear corridor rejects the candidate.
  robot_transform.translation.x = 0;
  map.data[80 * 160 + 60] = 254;
  publish(); rclcpp::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, ReverseGoalSelectionRejectsLongSidewaysAndChangedHeadingGoals)
{
  selectMotion("direct");
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}' path_planner='{chosen}'/>"
    "</BehaviorTree></root>", board);
  connect();
  for (const auto & p : std::vector<std::vector<double>>{{-1.02,0,0},{0,-.5,0},{-.5,0,.2}}) {
    auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
    goal.pose.position.x = p[0]; goal.pose.position.y = p[1];
    goal.pose.orientation.z = std::sin(p[2]/2); goal.pose.orientation.w = std::cos(p[2]/2);
    board->set("goal", goal);
    (void)tree.tickOnce();
    EXPECT_EQ(board->get<std::string>("chosen"), "Direct");
  }
}

TEST_F(FootprintBTTest, ForwardEscapeRequiresWholeGoalFacingTurnAtItsDestination)
{
  map.metadata.size_x = 240;
  map.metadata.origin.position.x = -3.0;
  map.data.assign(240 * 160, 0);
  for (unsigned int ix = 80; ix < 240; ++ix) {
    map.data[54 * 240 + ix] = 254;
    map.data[105 * 240 + ix] = 254;
  }
  selectMotion("forward");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -1.8; goal.pose.position.y = 1.0;
  board->set("goal", goal);
  connect();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
    << "A small angled opening in the aisle must not authorize advance into another dead end";
  auto begin = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='forward_begin' goal='{goal}' candidate_distance='0.30'/>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(begin.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(FootprintBTTest, OffsetRearGoalUsesContinuousCurveWithUnchangedFinalHeading)
{
  selectMotion("direct");
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x = -.7; goal.pose.position.y = .2;
  board->set("goal", goal);
  tree = factory.createTreeFromText(
    "<root BTCPP_format='4'><BehaviorTree ID='Main'>"
    "<PathFootprintClear motion='direct' goal='{goal}' checked_path='{path}' path_planner='{chosen}'/>"
    "</BehaviorTree></root>", board);
  connect();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<std::string>("chosen"), "DirectReverse");
  const auto path = board->get<nav_msgs::msg::Path>("path");
  ASSERT_GE(path.poses.size(), 80u);
  EXPECT_NEAR(tf2::getYaw(path.poses.front().pose.orientation), 0, 1e-8);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), 0, 1e-8);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, -.7);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, .2);
  for (size_t i=1; i<path.poses.size(); ++i) {
    const auto & a=path.poses[i-1].pose; const auto & b=path.poses[i].pose;
    const double heading=tf2::getYaw(a.orientation);
    EXPECT_LT((b.position.x-a.position.x)*std::cos(heading)+
      (b.position.y-a.position.y)*std::sin(heading),0);
  }
}
