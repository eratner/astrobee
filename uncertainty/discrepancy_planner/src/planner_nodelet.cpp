#include <discrepancy_planner/planner_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace discrepancy_planner {

PlannerNodelet::PlannerNodelet()
    : planner::PlannerImplementation("discrepancy", "Discrepancy planner"),
      tf_listener_(tf_buffer_),
      state_space_(nullptr),
      heuristic_(nullptr),
      search_(nullptr) {
  NODELET_DEBUG("Constructing discrepancy planner nodelet...");
}

PlannerNodelet::~PlannerNodelet() {
  NODELET_DEBUG("Destroying discrepancy planner nodelet...");
  if (state_space_) {
    delete state_space_;
    state_space_ = nullptr;
  }

  if (heuristic_) {
    delete heuristic_;
    heuristic_ = nullptr;
  }

  if (search_) {
    delete search_;
    search_ = nullptr;
  }
}

bool PlannerNodelet::InitializePlanner(ros::NodeHandle* nh) {
  cfg_.Initialize(GetPrivateHandle(), "mobility/planner_discrepancy.config");
  cfg_.Listen(boost::bind(&PlannerNodelet::ReconfigureCallback, this, _1));

  // Construct the state space.
  double goal_dist_thresh = cfg_.Get<double>("goal_dist_thresh");
  double goal_angle_thresh = cfg_.Get<double>("goal_angle_thresh");
  double x_disc = cfg_.Get<double>("x_disc");
  double y_disc = cfg_.Get<double>("y_disc");
  double z_disc = cfg_.Get<double>("z_disc");
  double yaw_disc = cfg_.Get<double>("yaw_disc");
  double prox_angle_disc = cfg_.Get<double>("prox_angle_disc");
  double dist_angle_disc = cfg_.Get<double>("dist_angle_disc");
  state_space_ = new FreeFlyerStateSpace(
    Eigen::Matrix<double, 4, 4>::Identity(), goal_dist_thresh,
    goal_angle_thresh,
    FreeFlyerStateSpace::Discretizer(
      {x_disc, y_disc, z_disc, yaw_disc, prox_angle_disc, dist_angle_disc}));

  // Load the motion primitives from the parameter server.
  XmlRpc::XmlRpcValue motion_primitives;
  if (nh->getParam("discrepancy_planner/motion_primitives",
                   motion_primitives)) {
    NODELET_WARN("Loading motion primitives from parameter server...");
    if (!state_space_->LoadMotionPrimitives(motion_primitives)) {
      NODELET_ERROR("Failed to load motion primitives");
      return false;
    }
  } else {
    NODELET_ERROR("Could not find the motion primitive parameters");
    return false;
  }

  // Construct the heuristic.
  // TODO Read the heuristic type from config
  double cost_per_meter = 10;  // TODO Read from config
  heuristic_ = new EuclideanHeuristic(state_space_, cost_per_meter);

  // Construct the search.
  search_ = new RTAA<FreeFlyerStateSpace::StateDim>(state_space_, heuristic_);
  double max_expansions = 75000;  // TODO Read from config
  search_->SetMaxExpansions(max_expansions);

  // TODO Read topic name from config
  std::string vis_topic = "/mob/planner_discrepancy/vis";
  vis_pub_ = nh->advertise<visualization_msgs::Marker>(vis_topic, 50);

  add_discrepancy_srv_ =
    nh->advertiseService("/mob/planner_discrepancy/add_discrepancy",
                         &PlannerNodelet::AddDiscrepancy, this);

  nominal_lin_vel_ = cfg_.Get<double>("nominal_lin_vel");
  nominal_ang_vel_ = cfg_.Get<double>("nominal_ang_vel");
  nominal_joint_vel_ = cfg_.Get<double>("nominal_joint_vel");

  return true;
}

bool PlannerNodelet::ReconfigureCallback(dynamic_reconfigure::Config& config) {
  if (!cfg_.Reconfigure(config)) return false;

  // TODO ...

  return true;
}

void PlannerNodelet::PlanCallback(ff_msgs::PlanGoal const& goal) {
  NODELET_DEBUG("Received new planning request!");

  ff_msgs::PlanResult result;

  if (goal.states.empty()) {
    NODELET_WARN("No goal pose specified!");
    result.response = ff_msgs::PlanResult::NOT_ENOUGH_STATES;
    return PlanResult(result);
  } else if (goal.states.size() > 2) {
    NODELET_WARN_STREAM("Too many goal poses specified (" << goal.states.size()
                                                          << ", expected 1)");
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  const auto& goal_pose = goal.states.back().pose;
  // ros::Time offset = goal.states.front().header.stamp;
  ros::Time offset = ros::Time::now();

  // Get the current pose of the robot.
  geometry_msgs::Pose start_pose;
  try {
    geometry_msgs::TransformStamped body_in_world =
      tf_buffer_.lookupTransform(std::string(FRAME_NAME_WORLD),
                                 std::string(FRAME_NAME_BODY), ros::Time(0));
    start_pose.position.x = body_in_world.transform.translation.x;
    start_pose.position.y = body_in_world.transform.translation.y;
    start_pose.position.z = body_in_world.transform.translation.z;
    start_pose.orientation = body_in_world.transform.rotation;
  } catch (tf2::TransformException& ex) {
    NODELET_ERROR_STREAM(
      "Failed to get pose of robot body in world: " << ex.what());
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  PublishDiscrepancyMarkers();

  // Set the goal state.
  // TODO Check if the goal state has changed-- if it has, need to clear the
  // state space
  Eigen::Matrix<double, 4, 4> goal_in_world =
    Eigen::Matrix<double, 4, 4>::Identity();
  Eigen::Quaterniond goal_in_world_rot(
    goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y,
    goal_pose.orientation.z);
  goal_in_world.block<3, 3>(0, 0) =
    goal_in_world_rot.normalized().toRotationMatrix();
  goal_in_world(0, 3) = goal_pose.position.x;
  goal_in_world(1, 3) = goal_pose.position.y;
  goal_in_world(2, 3) = goal_pose.position.z;
  state_space_->SetGoalPose(goal_in_world);
  PublishGoalMarker();

  double start_roll, start_pitch, start_yaw;
  QuaternionToRPY(start_pose.orientation.x, start_pose.orientation.y,
                  start_pose.orientation.z, start_pose.orientation.w,
                  start_roll, start_pitch, start_yaw);
  double start_yaw_fixed;
  if (!YawForZeroRollAndPitch(start_roll, start_pitch, start_yaw,
                              start_yaw_fixed)) {
    NODELET_ERROR_STREAM("Could not get the correct yaw of the robot");
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  } else {
    start_roll = 0;
    start_pitch = 0;
    start_yaw = start_yaw_fixed;
  }

  double goal_roll, goal_pitch, goal_yaw;
  QuaternionToRPY(goal_pose.orientation.x, goal_pose.orientation.y,
                  goal_pose.orientation.z, goal_pose.orientation.w, goal_roll,
                  goal_pitch, goal_yaw);
  NODELET_WARN_STREAM(
    "Planning from start (x, y, z) = ("
    << start_pose.position.x << ", " << start_pose.position.y << ", "
    << start_pose.position.z << "), (roll, pitch, yaw) = (" << start_roll
    << ", " << start_pitch << ", " << start_yaw << ") to goal (x, y, z) = ("
    << goal_pose.position.x << ", " << goal_pose.position.y << ", "
    << goal_pose.position.z << "), (roll, pitch, yaw) = (" << goal_roll << ", "
    << goal_pitch << ", " << goal_yaw << ")...");

  // Check if the robot is already at the goal state.
  if (state_space_->IsGoalPose(start_pose.position.x, start_pose.position.y,
                               start_pose.position.z, start_roll, start_pitch,
                               start_yaw, 0, 0)) {
    NODELET_WARN_STREAM("Robot already at the goal");
    result.response = ff_msgs::PlanResult::ALREADY_THERE;
    return PlanResult(result);
  }

  double start_x_on_graph, start_y_on_graph, start_z_on_graph,
    start_yaw_on_graph;
  MapPoseToSearchGraph(start_pose.position.x, start_pose.position.y,
                       start_pose.position.z, start_yaw, start_x_on_graph,
                       start_y_on_graph, start_z_on_graph, start_yaw_on_graph);

  auto start_state = state_space_->GetState(
    start_pose.position.x, start_pose.position.y, start_pose.position.z,
    start_roll, start_pitch, start_yaw, 0, 0);
  // auto start_state =
  //   state_space_->GetState(start_x_on_graph, start_y_on_graph, start_z_on_graph,
  //                          0, 0, start_yaw_on_graph, 0, 0);
  NODELET_WARN_STREAM("Planning from start state "
                      << *start_state << ", with heuristic cost-to-go "
                      << heuristic_->Value(start_state));

  // Search for a least-cost path to goal.
  auto goal_state = search_->Search(start_state);
  if (!goal_state || !goal_state->IsGoal()) {
    NODELET_ERROR("Search failed to find a path to goal");
    result.response = ff_msgs::PlanResult::NO_PATH_EXISTS;
    return PlanResult(result);
  }

  auto path_cost = search_->GetLastPathCost();
  auto path_and_actions = ellis_util::search::HeuristicSearch<
    FreeFlyerStateSpace::StateDim>::ReconstructPathWithActions(goal_state);
  auto path = std::get<0>(path_and_actions);
  auto actions = std::get<1>(path_and_actions);
  NODELET_WARN_STREAM("Found path to goal with cost " << path_cost << ":");
  for (int i = 0; i < path.size(); ++i) {
    NODELET_WARN_STREAM("  At state: " << *path[i]
                                       << ", taking action: " << actions[i]);
  }

  PublishPathMarker(path);

  auto traj =
    PathToTrajectory(path, start_pose.position.x, start_pose.position.y,
                     start_pose.position.z, start_yaw, 0, 0);

  double traj_start_time;
  if (!traj.GetStartTime(traj_start_time)) {
    NODELET_ERROR_STREAM("Could not get the trajectory start time");
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  double traj_end_time;
  if (!traj.GetEndTime(traj_end_time)) {
    NODELET_ERROR_STREAM("Could not get the trajectory end time");
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  result.response = ff_msgs::PlanResult::SUCCESS;
  last_trajectory_ = traj;
  last_actions_ = actions;

  // TODO Assign the trajectory
  result.segment.clear();

  // TODO Read this from the config
  double time_between_waypoints = 0.1;
  double time = traj_start_time;
  while (time <= traj_end_time) {
    std::array<double, kTrajectoryDim> pos, vel, acc;
    if (traj.Get(time, pos, 0) && traj.Get(time, vel, 1) &&
        traj.Get(time, acc, 2)) {
      ff_msgs::ControlState waypoint;
      waypoint.when = offset + ros::Duration(time);

      // Waypoint position.
      waypoint.pose.position.x = pos[FreeFlyerStateSpace::X];
      waypoint.pose.position.y = pos[FreeFlyerStateSpace::Y];
      waypoint.pose.position.z = pos[FreeFlyerStateSpace::Z];
      auto orien = RPYToQuaternion(0, 0, pos[FreeFlyerStateSpace::YAW]);
      waypoint.pose.orientation.x = orien.x();
      waypoint.pose.orientation.y = orien.y();
      waypoint.pose.orientation.z = orien.z();
      waypoint.pose.orientation.w = orien.w();

      // Waypoint velocity.
      waypoint.twist.linear.x = vel[FreeFlyerStateSpace::X];
      waypoint.twist.linear.y = vel[FreeFlyerStateSpace::Y];
      waypoint.twist.linear.z = vel[FreeFlyerStateSpace::Z];
      waypoint.twist.angular.x = 0;
      waypoint.twist.angular.y = 0;
      waypoint.twist.angular.z = vel[FreeFlyerStateSpace::YAW];

      // Waypoint acceleration.
      waypoint.accel.linear.x = acc[FreeFlyerStateSpace::X];
      waypoint.accel.linear.y = acc[FreeFlyerStateSpace::Y];
      waypoint.accel.linear.z = acc[FreeFlyerStateSpace::Z];
      waypoint.accel.angular.x = 0;
      waypoint.accel.angular.y = 0;
      waypoint.accel.angular.z = acc[FreeFlyerStateSpace::YAW];

      result.segment.push_back(waypoint);
    } else
      NODELET_ERROR_STREAM("Could not get waypoint in trajectory at time "
                           << time);

    time += time_between_waypoints;
  }

  PlanResult(result);
}

void PlannerNodelet::MapPoseToSearchGraph(double x_in, double y_in, double z_in,
                                          double yaw_in, double& x_out,
                                          double& y_out, double& z_out,
                                          double& yaw_out) const {
  // TODO Make parameters/read from motion primitives
  const int units_per_transition_x = 10;
  const int units_per_transition_y = 10;
  const int units_per_transition_z = 10;
  const int units_per_transition_yaw = 10;

  // Map x onto the search graph.
  x_out =
    MapToSearchGraph(x_in, FreeFlyerStateSpace::X, units_per_transition_x);

  // Map y onto the search graph.
  y_out =
    MapToSearchGraph(y_in, FreeFlyerStateSpace::Y, units_per_transition_y);

  // Map z onto the search graph.
  z_out =
    MapToSearchGraph(z_in, FreeFlyerStateSpace::Z, units_per_transition_z);

  // Map yaw onto the search graph.
  yaw_out = MapToSearchGraph(yaw_in, FreeFlyerStateSpace::YAW,
                             units_per_transition_yaw);
}

double PlannerNodelet::MapToSearchGraph(
  double value, FreeFlyerStateSpace::VariableIndex variable,
  int units_per_transition) const {
  int value_disc = state_space_->GetDiscretizer().Discretize(value, variable);
  value_disc = (value_disc / units_per_transition) * units_per_transition;
  // value_disc =
  //   static_cast<int>(std::round(1.0 * value_disc /
  //                               static_cast<double>(units_per_transition))) *
  //   units_per_transition;
  return state_space_->GetDiscretizer().Undiscretize(value_disc, variable);
}

PolynomialTrajectory<kTrajectoryDim> PlannerNodelet::PathToTrajectory(
  const std::vector<FreeFlyerStateSpace::State*>& path, double start_x,
  double start_y, double start_z, double start_yaw, double start_prox_angle,
  double start_dist_angle, double start_time_sec) {
  std::vector<std::array<double, kTrajectoryDim>> waypoints;
  std::vector<double> waypoint_times;

  double time = start_time_sec;

  for (int i = 0; i < path.size() + 1; ++i) {
    double waypoint_x, waypoint_y, waypoint_z, waypoint_yaw,
      waypoint_prox_angle, waypoint_dist_angle, unused;
    if (i == 0) {
      // Ensure that the trajectory begins from the robot's exact start pose.
      waypoint_x = start_x;
      waypoint_y = start_y;
      waypoint_z = start_z;
      waypoint_yaw = start_yaw;
      waypoint_prox_angle = start_prox_angle;
      waypoint_dist_angle = start_dist_angle;
    } else {
      state_space_->GetPose(path[i - 1]->GetVariables(), waypoint_x, waypoint_y,
                            waypoint_z, unused, unused, waypoint_yaw,
                            waypoint_prox_angle, waypoint_dist_angle);
    }
    waypoints.push_back({waypoint_x, waypoint_y, waypoint_z, waypoint_yaw,
                         waypoint_prox_angle, waypoint_dist_angle});

    if (i > 0) {
      const auto& last_waypoint = waypoints[waypoints.size() - 2];
      const auto& waypoint = waypoints[waypoints.size() - 1];
      time += GetTimeBetweenWaypoints(last_waypoint, waypoint);
    }

    waypoint_times.push_back(time);
  }

  std::array<double, kTrajectoryDim> vels = {
    nominal_lin_vel_,    // x
    nominal_lin_vel_,    // y
    nominal_lin_vel_,    // z
    nominal_ang_vel_,    // yaw
    nominal_joint_vel_,  // proximal angle
    nominal_joint_vel_   // distal angle
  };

  NODELET_INFO("Trajectory waypoints: ");
  for (int i = 0; i < waypoints.size(); ++i) {
    const auto& waypoint = waypoints[i];
    NODELET_INFO_STREAM("  pos: ("
                        << waypoint[FreeFlyerStateSpace::X] << ", "
                        << waypoint[FreeFlyerStateSpace::Y] << ", "
                        << waypoint[FreeFlyerStateSpace::Z]
                        << "), yaw: " << waypoint[FreeFlyerStateSpace::YAW]
                        << ", prox angle: "
                        << waypoint[FreeFlyerStateSpace::PROX_ANGLE]
                        << ", dist angle: "
                        << waypoint[FreeFlyerStateSpace::DIST_ANGLE]
                        << ", time: " << waypoint_times[i]);
  }

  return PolynomialTrajectory<kTrajectoryDim>(waypoints, waypoint_times, vels);
}

double PlannerNodelet::GetTimeBetweenWaypoints(
  const std::array<double, kTrajectoryDim>& first_waypoint,
  const std::array<double, kTrajectoryDim>& second_waypoint) const {
  double lin_dist_between_waypoints =
    std::sqrt(std::pow(second_waypoint[FreeFlyerStateSpace::X] -
                         first_waypoint[FreeFlyerStateSpace::X],
                       2) +
              std::pow(second_waypoint[FreeFlyerStateSpace::Y] -
                         first_waypoint[FreeFlyerStateSpace::Y],
                       2) +
              std::pow(second_waypoint[FreeFlyerStateSpace::Z] -
                         first_waypoint[FreeFlyerStateSpace::Z],
                       2));
  double ang_dist_between_waypoints =
    std::abs(angles::shortest_angular_distance(
      first_waypoint[FreeFlyerStateSpace::YAW],
      second_waypoint[FreeFlyerStateSpace::YAW]));
  double joint_dist_between_waypoints =
    std::max(std::abs(angles::shortest_angular_distance(
               first_waypoint[FreeFlyerStateSpace::PROX_ANGLE],
               second_waypoint[FreeFlyerStateSpace::PROX_ANGLE])),
             std::abs(angles::shortest_angular_distance(
               first_waypoint[FreeFlyerStateSpace::DIST_ANGLE],
               second_waypoint[FreeFlyerStateSpace::DIST_ANGLE])));
  double time =
    std::max(std::max(lin_dist_between_waypoints / nominal_lin_vel_,
                      ang_dist_between_waypoints / nominal_ang_vel_),
             joint_dist_between_waypoints / nominal_joint_vel_);
  // TODO Make this a parameter.
  if (time < 0.5) time = 0.5;

  return time;
}

bool PlannerNodelet::AddDiscrepancy(std_srvs::Trigger::Request& req,
                                    std_srvs::Trigger::Response& res) {
  // Get the robot's current pose.
  geometry_msgs::Pose pose;
  try {
    geometry_msgs::TransformStamped body_in_world =
      tf_buffer_.lookupTransform(std::string(FRAME_NAME_WORLD),
                                 std::string(FRAME_NAME_BODY), ros::Time(0));
    pose.position.x = body_in_world.transform.translation.x;
    pose.position.y = body_in_world.transform.translation.y;
    pose.position.z = body_in_world.transform.translation.z;
    pose.orientation = body_in_world.transform.rotation;
  } catch (tf2::TransformException& ex) {
    NODELET_ERROR_STREAM(
      "Failed to get pose of robot body in world: " << ex.what());
    res.success = false;
    return false;
  }

  NODELET_WARN_STREAM("Attempting to add discrepancy at robot pos: ("
                      << pose.position.x << ", " << pose.position.y << ", "
                      << pose.position.z
                      << "), yaw: " << tf::getYaw(pose.orientation) << "...");

  // Search for the point in the trajectory at which the discrepancy occured,
  // based on where the robot currently is.
  const auto& segments = last_trajectory_.GetSegments();
  if (segments.size() <= 1) {
    NODELET_ERROR("Last trajectory was empty");
    return false;
  }

  // Get the start and end times of the trajectory.
  // NOTE Here we assume the discrepancy does not occur on the trajectory's
  // first segment (the first segment connects the start pose of the robot to
  // the first pose on the path, to account for discretizatione errors).
  double start_time = segments[1].start_time_;
  double end_time = segments.back().end_time_;

  // TODO Read from config
  double time_between_waypoints = 0.1;

  std::vector<double> waypoint_times;
  double time = start_time + time_between_waypoints;
  while (time <= end_time) {
    waypoint_times.push_back(time);
    time += time_between_waypoints;
  }

  int best_segment_index = -1;
  double best_dist = 1e9;

  for (auto waypoint_time : waypoint_times) {
    std::array<double, kTrajectoryDim> pos;
    int segment_index = -1;
    if (last_trajectory_.Get(waypoint_time, pos, 0, &segment_index)) {
      double dist =
        std::sqrt(std::pow(pos[FreeFlyerStateSpace::X] - pose.position.x, 2) +
                  std::pow(pos[FreeFlyerStateSpace::Y] - pose.position.y, 2) +
                  std::pow(pos[FreeFlyerStateSpace::Z] - pose.position.z, 2));
      double ang_dist = std::abs(angles::shortest_angular_distance(
        tf::getYaw(pose.orientation), pos[FreeFlyerStateSpace::YAW]));
      NODELET_DEBUG_STREAM("  pos: ("
                           << pos[FreeFlyerStateSpace::X] << ", "
                           << pos[FreeFlyerStateSpace::Y] << ", "
                           << pos[FreeFlyerStateSpace::Z]
                           << "), yaw: " << pos[FreeFlyerStateSpace::YAW]
                           << ", dist: " << dist << ", ang dist: " << ang_dist
                           << ", segment: " << segment_index
                           << ", best dist: " << best_dist
                           << ", best segment: " << best_segment_index
                           << ", time: " << waypoint_time);
      // TODO Also check arm joint angles
      if (dist + ang_dist < best_dist) {
        best_dist = dist + ang_dist;
        best_segment_index = segment_index;
      }
    } else
      NODELET_ERROR_STREAM("Could not get waypoint in trajectory at time "
                           << waypoint_time);
  }

  if (best_segment_index - 1 < 0 ||
      best_segment_index - 1 >= last_actions_.size()) {
    NODELET_ERROR_STREAM(
      "Discrepancy occurred at segment "
      << best_segment_index
      << ", which does not correspond to any action along the planned path");
    res.success = false;
    return false;
  }
  auto action = last_actions_[best_segment_index - 1];
  NODELET_WARN_STREAM("Discrepancy at action "
                      << action << " (best segment index is "
                      << best_segment_index << ", with combined distance "
                      << best_dist << ")");

  // Add a discrepancy neighborhood around the current state and action to the
  // planner's state space.
  FreeFlyerStateSpace::DiscrepancyNeighborhood discrepancy;
  discrepancy.penalty_ = cfg_.Get<double>("discrepancy_penalty");
  discrepancy.radius_.pos_ = cfg_.Get<double>("discrepancy_radius_pos");
  discrepancy.radius_.orien_ = cfg_.Get<double>("discrepancy_radius_orien");
  discrepancy.radius_.prox_angle_ =
    cfg_.Get<double>("discrepancy_radius_prox_angle");
  discrepancy.radius_.dist_angle_ =
    cfg_.Get<double>("discrepancy_radius_dist_angle");

  discrepancy.action_ = action;
  discrepancy.state_.x_ = pose.position.x;
  discrepancy.state_.y_ = pose.position.y;
  discrepancy.state_.z_ = pose.position.z;
  discrepancy.state_.yaw_ = tf::getYaw(pose.orientation);
  discrepancy.state_.prox_angle_ = 0;  // TODO
  discrepancy.state_.dist_angle_ = 0;  // TODO
  state_space_->AddDiscrepancy(discrepancy);

  res.success = true;
  return true;
}

void PlannerNodelet::PublishGoalMarker() {
  if (!state_space_) {
    NODELET_WARN_STREAM(
      "[PublishGoalMarker] State space has not yet been initialized");
    return;
  }

  visualization_msgs::Marker goal_msg;
  goal_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
  goal_msg.ns = "discrepancy_planner/goal";
  goal_msg.id = 0;
  goal_msg.type = visualization_msgs::Marker::SPHERE;

  double goal_x, goal_y, goal_z, unused;
  state_space_->GetGoalPose(goal_x, goal_y, goal_z, unused, unused, unused);

  goal_msg.pose.position.x = goal_x;
  goal_msg.pose.position.y = goal_y;
  goal_msg.pose.position.z = goal_z;
  goal_msg.pose.orientation.x = 0;
  goal_msg.pose.orientation.y = 0;
  goal_msg.pose.orientation.z = 0;
  goal_msg.pose.orientation.w = 1;
  goal_msg.color.r = 0;
  goal_msg.color.g = 1;
  goal_msg.color.b = 0;
  goal_msg.color.a = 0.75;
  auto goal_dist_thresh = state_space_->GetGoalDistThresh();
  goal_msg.scale.x = goal_dist_thresh;
  goal_msg.scale.y = goal_dist_thresh;
  goal_msg.scale.z = goal_dist_thresh;
  vis_pub_.publish(goal_msg);
}

void PlannerNodelet::PublishPathMarker(
  const std::vector<FreeFlyerStateSpace::State*>& path) {
  visualization_msgs::Marker path_msg;
  path_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
  path_msg.ns = "discrepancy_planner/path";
  path_msg.id = 0;
  path_msg.type = visualization_msgs::Marker::LINE_STRIP;

  path_msg.pose.position.x = 0;
  path_msg.pose.position.y = 0;
  path_msg.pose.position.z = 0;
  path_msg.pose.orientation.x = 0;
  path_msg.pose.orientation.y = 0;
  path_msg.pose.orientation.z = 0;
  path_msg.pose.orientation.w = 1;
  path_msg.color.r = 1;
  path_msg.color.g = 1;
  path_msg.color.b = 0;
  path_msg.color.a = 0.5;

  path_msg.scale.x = 0.05;

  for (auto state : path) {
    geometry_msgs::Point p;
    double unused;
    state_space_->GetPose(state->GetVariables(), p.x, p.y, p.z, unused, unused,
                          unused, unused, unused);
    path_msg.points.push_back(p);
  }

  vis_pub_.publish(path_msg);
}

void PlannerNodelet::PublishDiscrepancyMarkers() {
  const auto& discrepancies = state_space_->GetDiscrepancies();
  for (int i = 0; i < discrepancies.size(); ++i) {
    const auto& discrepancy = discrepancies[i];

    visualization_msgs::Marker discrepancy_msg;
    discrepancy_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    discrepancy_msg.ns = "discrepancy_planner/discrepancy";
    discrepancy_msg.id = i;
    discrepancy_msg.type = visualization_msgs::Marker::SPHERE;

    discrepancy_msg.pose.position.x = discrepancy.state_.x_;
    discrepancy_msg.pose.position.y = discrepancy.state_.y_;
    discrepancy_msg.pose.position.z = discrepancy.state_.z_;
    discrepancy_msg.pose.orientation.x = 0;
    discrepancy_msg.pose.orientation.y = 0;
    discrepancy_msg.pose.orientation.z = 0;
    discrepancy_msg.pose.orientation.w = 1;
    discrepancy_msg.color.r = 0;
    discrepancy_msg.color.g = 0;
    discrepancy_msg.color.b = 1;
    discrepancy_msg.color.a = 0.25;
    discrepancy_msg.scale.x = discrepancy.radius_.pos_;
    discrepancy_msg.scale.y = discrepancy.radius_.pos_;
    discrepancy_msg.scale.z = discrepancy.radius_.pos_;
    vis_pub_.publish(discrepancy_msg);
  }
}

}  // namespace discrepancy_planner

PLUGINLIB_EXPORT_CLASS(discrepancy_planner::PlannerNodelet, nodelet::Nodelet);
