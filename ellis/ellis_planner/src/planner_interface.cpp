// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/planner_interface.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <string>
#include <algorithm>

namespace ellis_planner {

PlannerInterface::PlannerInterface()
    : planner::PlannerImplementation("ellis", "Ellis's experimental planner"),
      tf_listener_(tf_buffer_),
      search_(&env_) {
  NODELET_DEBUG("Constructing ellis planner nodelet...");
}

PlannerInterface::~PlannerInterface() { NODELET_DEBUG("Destroying ellis planner nodelet..."); }

bool PlannerInterface::InitializePlanner(ros::NodeHandle* nh) {
  cfg_.Initialize(GetPrivateHandle(), "mobility/ellis_planner.config");
  cfg_.Listen(boost::bind(&PlannerInterface::ReconfigureCallback, this, _1));

  // TODO(eratner) Read topic name from config
  std::string vis_topic = "/mob/ellis_planner/vis";
  vis_pub_ = nh->advertise<visualization_msgs::Marker>(vis_topic, 50);

  // report_execution_error_srv_ =
  //   nh->advertiseService("/mob/ellis_planner/report_execution_error", &PlannerInterface::ReportExecutionError, this);
  report_execution_error_srv_ =
    nh->advertiseService("/mob/ellis_planner/report_execution_error", &PlannerInterface::ReportExecutionError, this);

  clear_execution_errors_srv_ =
    nh->advertiseService("/mob/ellis_planner/clear_execution_errors", &PlannerInterface::ClearExecutionErrors, this);

  nominal_lin_vel_ = cfg_.Get<double>("nominal_lin_vel");
  nominal_ang_vel_ = cfg_.Get<double>("nominal_ang_vel");

  std::vector<Environment::Action> actions;
  XmlRpc::XmlRpcValue action_info;
  if (nh->getParam("ellis_planner/actions", action_info)) {
    unsigned int num_actions = action_info.size();
    for (unsigned int i = 0; i < num_actions; ++i) {
      std::string action_prefix = "ellis_planner/actions/a" + std::to_string(i) + "/";

      Environment::Action action;

      if (!nh->getParam(action_prefix + "name", action.name_)) {
        NODELET_WARN_STREAM("Action " << i << " has no name!");
        continue;
      }

      action.change_in_x_ = nh->param<double>(action_prefix + "change_in_x", 0.0);
      action.change_in_y_ = nh->param<double>(action_prefix + "change_in_y", 0.0);
      action.change_in_yaw_ = nh->param<double>(action_prefix + "change_in_yaw", 0.0);

      if (!nh->getParam(action_prefix + "cost", action.cost_)) {
        NODELET_WARN_STREAM("Action " << i << " has no cost!");
        continue;
      }

      actions.push_back(action);
    }
  } else {
    NODELET_WARN("Failed to get actions from ellis_planner/actions");
  }
  env_.SetActions(actions);

  env_.SetExecutionErrorNeighborhoodParameters(Environment::ExecutionErrorNeighborhoodParameters(
    cfg_.Get<double>("exec_error_state_radius_pos"), cfg_.Get<double>("exec_error_state_radius_yaw"),
    cfg_.Get<double>("exec_error_action_radius"), cfg_.Get<double>("exec_error_penalty")));
  NODELET_ERROR_STREAM("Params: " << env_.GetExecutionErrorNeighborhoodParameters());

  return true;
}

bool PlannerInterface::ReconfigureCallback(dynamic_reconfigure::Config& config) {
  if (!cfg_.Reconfigure(config)) return false;

  // Update the execution error neighborhood parameters.
  env_.SetExecutionErrorNeighborhoodParameters(Environment::ExecutionErrorNeighborhoodParameters(
    cfg_.Get<double>("exec_error_state_radius_pos"), cfg_.Get<double>("exec_error_state_radius_yaw"),
    cfg_.Get<double>("exec_error_action_radius"), cfg_.Get<double>("exec_error_penalty")));
  NODELET_ERROR_STREAM("Params: " << env_.GetExecutionErrorNeighborhoodParameters());

  return true;
}

void PlannerInterface::PlanCallback(const ff_msgs::PlanGoal& goal) {
  NODELET_DEBUG("Received new planning request!");

  ff_msgs::PlanResult result;

  // Set the boundaries on the state space.
  std::vector<ff_msgs::Zone> zones;
  if (!GetZones(zones)) {
    NODELET_ERROR("Could not get the zones!");
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }
  // TODO(eratner) For now, only support 1 KEEPIN zone
  bool keepin_zone_found = false;
  for (const auto& zone : zones) {
    if (zone.type == ff_msgs::Zone::KEEPIN) {
      if (!keepin_zone_found) {
        keepin_zone_found = true;
        // Set bounds on the environment used in planning.
        env_.SetBounds(zone.min.x, zone.max.x, zone.min.y, zone.max.y);
      } else {
        NODELET_WARN_STREAM("Already found one KEEPIN zone, so skipping zone \"" << zone.name << "\" with index "
                                                                                 << zone.index);
      }
    } else {
      NODELET_WARN_STREAM("Zone \"" << zone.name << "\" with index " << zone.index << " and has unsupported type "
                                    << zone.type << ", so ignoring...");
    }
  }

  if (!keepin_zone_found) NODELET_WARN("No KEEPIN zone found, so no boundary on state space set");

  if (goal.states.empty()) {
    NODELET_WARN("No goal pose specified!");
    result.response = ff_msgs::PlanResult::NOT_ENOUGH_STATES;
    return PlanResult(result);
  } else if (goal.states.size() > 2) {
    NODELET_WARN_STREAM("Too many goal poses specified (" << goal.states.size() << ", expected 1)");
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  const auto& goal_pose = goal.states.back().pose;
  double goal_yaw = tf2::getYaw(goal_pose.orientation);
  ros::Time offset = ros::Time::now();

  // TODO(eratner) Check if goal is out-of-bounds

  double start_x = 0.0, start_y = 0.0, start_z = 0.0, start_yaw = 0.0;
  if (!GetPose(start_x, start_y, start_z, start_yaw)) {
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  PublishPoseMarker(start_x, start_y, start_z, start_yaw, "ellis/start");
  PublishPoseMarker(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, goal_yaw, "ellis/goal");

  // TODO(eratner) This may be inefficient, but makes sure old search data is not carried over into subsequent searches
  env_.Clear();

  env_.SetGoal(goal_pose.position.x, goal_pose.position.y, goal_yaw);
  std::vector<ellis_planner::State::Ptr> path;
  auto start_state = env_.GetState(start_x, start_y, start_yaw);
  if (!search_.Run(start_state, path)) {
    NODELET_ERROR_STREAM("Could not find a path to the goal, with start ("
                         << start_x << ", " << start_y << ", " << start_yaw << ") and goal (" << goal_pose.position.x
                         << ", " << goal_pose.position.y << ", " << goal_yaw << ")");
    NODELET_ERROR(" Actions: ");
    for (const auto& action : env_.GetActions()) NODELET_ERROR_STREAM("  " << action);
    result.response = ff_msgs::PlanResult::BAD_ARGUMENTS;
    return PlanResult(result);
  }

  DeletePathMarkers();
  PublishPathMarkers(path);

  NODELET_ERROR("-----");
  NODELET_ERROR_STREAM("state: " << *start_state);
  for (const auto& nbhd : env_.GetExecutionErrorNeighborhoods()) {
    NODELET_ERROR_STREAM("  nbhd: " << nbhd);
    for (const auto& action : env_.GetActions()) {
      NODELET_ERROR_STREAM(
        "    action: " << action << ", contains? "
                       << (nbhd.Contains(start_state, action, env_.GetExecutionErrorNeighborhoodParameters()) ? "YES"
                                                                                                              : "NO"));
    }
  }

  std::vector<Waypoint> waypoints;
  for (const auto state : path) {
    waypoints.push_back({state->GetX(), state->GetY(), state->GetYaw()});
  }
  auto traj = ToTrajectory(waypoints, 0);

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

  result.segment.clear();

  // TODO(eratner) Read this from the config
  double time_between_waypoints = 0.1;
  double time = traj_start_time;
  while (time <= traj_end_time) {
    Waypoint pos, vel, acc;
    if (traj.Get(time, pos, 0) && traj.Get(time, vel, 1) && traj.Get(time, acc, 2)) {
      ff_msgs::ControlState waypoint;
      waypoint.when = offset + ros::Duration(time);

      // Waypoint position.
      waypoint.pose.position.x = pos[0];
      waypoint.pose.position.y = pos[1];
      waypoint.pose.position.z = start_z;
      tf2::Quaternion orien;
      orien.setRPY(0, 0, pos[2]);
      waypoint.pose.orientation.x = orien.x();
      waypoint.pose.orientation.y = orien.y();
      waypoint.pose.orientation.z = orien.z();
      waypoint.pose.orientation.w = orien.w();

      // Waypoint velocity.
      waypoint.twist.linear.x = vel[0];
      waypoint.twist.linear.y = vel[1];
      waypoint.twist.linear.z = 0;
      waypoint.twist.angular.x = 0;
      waypoint.twist.angular.y = 0;
      waypoint.twist.angular.z = vel[2];

      // Waypoint acceleration.
      waypoint.accel.linear.x = acc[0];
      waypoint.accel.linear.y = acc[1];
      waypoint.accel.linear.z = 0;
      waypoint.accel.angular.x = 0;
      waypoint.accel.angular.y = 0;
      waypoint.accel.angular.z = acc[2];

      result.segment.push_back(waypoint);
    } else {
      NODELET_ERROR_STREAM("Could not get waypoint in trajectory at time " << time);
    }

    time += time_between_waypoints;
  }

  last_trajectory_ = result.segment;

  PlanResult(result);
}

PolynomialTrajectory<kStateDim> PlannerInterface::ToTrajectory(const std::vector<Waypoint>& waypoints,
                                                               double start_time_sec) {
  std::vector<double> times = {start_time_sec};
  std::vector<Waypoint> vels = {{0, 0, 0}};
  double time = start_time_sec;

  for (int i = 1; i < waypoints.size(); ++i) {
    const auto& last_waypoint = waypoints[i - 1];
    const auto& waypoint = waypoints[i];
    time += GetTimeBetweenWaypoints(last_waypoint, waypoint);
    times.push_back(time);

    if (i < waypoints.size() - 1) {
      std::array<int, 3> vel_dir = {0, 0, 0};
      for (int j = 0; j < 3; ++j) {
        double diff = waypoint[j] - last_waypoint[j];
        if (std::abs(diff) > 1e-6) {
          if (diff > 0.0)
            vel_dir[j] = 1;
          else
            vel_dir[j] = -1;
        }
      }

      const auto& next_waypoint = waypoints[i + 1];
      std::array<int, 3> next_vel_dir = {0, 0, 0};
      for (int j = 0; j < 3; ++j) {
        double diff = next_waypoint[j] - waypoint[j];
        if (std::abs(diff) > 1e-6) {
          if (diff > 0.0)
            next_vel_dir[j] = 1;
          else
            next_vel_dir[j] = -1;
        }
      }

      double vel_x = 0.0, vel_y = 0.0, vel_yaw = 0.0;
      if (vel_dir[0] == next_vel_dir[0] && vel_dir[1] == next_vel_dir[1] && vel_dir[2] == next_vel_dir[2]) {
        vel_x = static_cast<double>(vel_dir[0]) * nominal_lin_vel_;
        vel_y = static_cast<double>(vel_dir[1]) * nominal_lin_vel_;
        vel_yaw = static_cast<double>(vel_dir[2]) * nominal_ang_vel_;
      }

      vels.push_back({vel_x,      // x
                      vel_y,      // y
                      vel_yaw});  // yaw
    } else {
      vels.push_back({0, 0, 0});
    }
  }

  NODELET_INFO("Trajectory waypoints: ");
  for (int i = 0; i < waypoints.size(); ++i) {
    const auto& waypoint = waypoints[i];
    NODELET_INFO_STREAM("  pos: (" << waypoint[0] << ", " << waypoint[1] << "), yaw: " << waypoint[2]
                                   << ", time: " << times[i]);
  }

  return PolynomialTrajectory<kStateDim>(waypoints, vels, times);
}

double PlannerInterface::GetTimeBetweenWaypoints(const Waypoint& first_waypoint,
                                                 const Waypoint& second_waypoint) const {
  double dist_between_waypoints = std::sqrt(std::pow(second_waypoint[0] - first_waypoint[0], 2) +
                                            std::pow(second_waypoint[1] - first_waypoint[1], 2));
  double ang_between_waypoints = std::abs(AngularDist(first_waypoint[2], second_waypoint[2]));
  double time = std::max(dist_between_waypoints / nominal_lin_vel_, ang_between_waypoints / nominal_ang_vel_);
  // TODO(eratner) Make this a parameter.
  if (time < 0.5) time = 0.5;

  return time;
}

bool PlannerInterface::GetPose(double& x, double& y, double& z, double& yaw) {
  geometry_msgs::TransformStamped world_to_body;
  try {
    world_to_body =
      tf_buffer_.lookupTransform(std::string(FRAME_NAME_WORLD), std::string(FRAME_NAME_BODY), ros::Time(0));
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR_STREAM("[ExperimentManager] Error: " << ex.what());
    return false;
  }

  x = world_to_body.transform.translation.x;
  y = world_to_body.transform.translation.y;
  z = world_to_body.transform.translation.z;
  yaw = tf2::getYaw(world_to_body.transform.rotation);
  return true;
}

void PlannerInterface::PublishPoseMarker(double x, double y, double z, double yaw, const std::string& name,
                                         bool show_name, double r, double g, double b, double a) {
  visualization_msgs::Marker pose_msg;
  pose_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
  pose_msg.ns = name;
  pose_msg.id = 0;
  pose_msg.type = visualization_msgs::Marker::ARROW;
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;
  tf2::Quaternion orien;
  orien.setRPY(0, 0, yaw);
  pose_msg.pose.orientation.x = orien.x();
  pose_msg.pose.orientation.y = orien.y();
  pose_msg.pose.orientation.z = orien.z();
  pose_msg.pose.orientation.w = orien.w();
  pose_msg.color.r = r;
  pose_msg.color.g = g;
  pose_msg.color.b = b;
  pose_msg.color.a = a;
  pose_msg.scale.x = 0.2;
  pose_msg.scale.y = 0.05;
  pose_msg.scale.z = 0.05;
  vis_pub_.publish(pose_msg);

  if (show_name) {
    visualization_msgs::Marker name_msg;
    name_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    name_msg.ns = name;
    name_msg.id = 1;
    name_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    name_msg.pose = pose_msg.pose;
    name_msg.pose.position.z += 0.05;
    name_msg.color.r = 1.0;
    name_msg.color.g = 1.0;
    name_msg.color.b = 1.0;
    name_msg.color.a = 1.0;
    name_msg.scale.z = 0.1;
    name_msg.text = name;
    vis_pub_.publish(name_msg);
  }
}

// bool PlannerInterface::ReportExecutionError(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
//   // Get the robot's current pose.
//   double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
//   if (!GetPose(x, y, z, yaw)) {
//     NODELET_ERROR("Failed to get pose of robot!");
//     res.success = false;
//     return false;
//   }

//   // Find the closest waypoint to the robot's current pose.
//   int closest_index = -1;
//   double closest_dist = 1e9;
//   double closest_angle = 1e9;
//   for (int i = 0; i < last_trajectory_.size(); ++i) {
//     const auto& waypoint = last_trajectory_[i];
//     double dist = std::sqrt(std::pow(waypoint.pose.position.x - x, 2) + std::pow(waypoint.pose.position.y - y, 2));
//     double waypoint_yaw = tf2::getYaw(waypoint.pose.orientation);
//     double angle = std::abs(AngularDist(tf2::getYaw(waypoint.pose.orientation), yaw));
//     if (dist < closest_dist && (std::abs(angle - closest_angle) < 1e-3 || angle < closest_angle)) {
//       closest_dist = dist;
//       closest_angle = angle;
//       closest_index = i;
//     }
//   }

//   if (closest_index < 0) {
//     NODELET_ERROR("Failed to find closest waypoint in last trajectory!");
//     return false;
//   }

//   auto waypoint = last_trajectory_[closest_index];
//   double n = std::sqrt(std::pow(waypoint.twist.linear.x, 2) + std::pow(waypoint.twist.linear.y, 2));
//   if (std::abs(n) < 1e-6) {
//     NODELET_WARN_STREAM("Waypoint at index " << closest_index << " has zero velocity, so using waypoint at index "
//                                              << closest_index - 1 << " or index " << closest_index + 1);
//     waypoint = last_trajectory_[(closest_index > 0 ? closest_index - 1 : closest_index + 1)];
//     n = std::sqrt(std::pow(waypoint.twist.linear.x, 2) + std::pow(waypoint.twist.linear.y, 2));
//     if (std::abs(n) < 1e-6) {
//       NODELET_ERROR_STREAM("Waypoints at indices " << closest_index << " and "
//                                                    << (closest_index > 0 ? closest_index - 1 : closest_index + 1)
//                                                    << " both have velocity of zero!");
//       return false;
//     }
//   }

//   double yaw_dir = 0.0;
//   if (std::abs(waypoint.twist.angular.z) > 1e-6) {
//     if (waypoint.twist.angular.z > 0.0)
//       yaw_dir = 1.0;
//     else
//       yaw_dir = -1.0;
//   }
//   env_.AddExecutionErrorNeighborhood(Environment::ExecutionErrorNeighborhood(x, y, yaw, waypoint.twist.linear.x / n,
//                                                                              waypoint.twist.linear.y / n, yaw_dir));
//   NODELET_ERROR_STREAM("Exec error at (" << x << ", " << y << ", " << yaw << "), action lin vel: ("
//                                          << waypoint.twist.linear.x << ", " << waypoint.twist.linear.y
//                                          << "), ang vel: " << waypoint.twist.angular.z << ", n: " << n);

//   PublishExecutionErrorNeighborhoodMarkers();

//   res.success = true;
//   return true;
// }

bool PlannerInterface::ReportExecutionError(ReportExecutionError::Request& req, ReportExecutionError::Response& res) {
  NODELET_ERROR_STREAM("Exec error at (" << req.x << ", " << req.y << ", " << req.yaw << ") with action ("
                                         << req.action_dir_x << ", " << req.action_dir_y << ", " << req.action_dir_yaw
                                         << ")");
  env_.AddExecutionErrorNeighborhood(Environment::ExecutionErrorNeighborhood(req));
  PublishExecutionErrorNeighborhoodMarkers();

  res.success = true;
  return true;
}

void PlannerInterface::PublishExecutionErrorNeighborhoodMarkers() {
  const auto& nbhds = env_.GetExecutionErrorNeighborhoods();
  for (int i = 0; i < nbhds.size(); ++i) {
    const auto& nbhd = nbhds[i];

    // Visualize the state position neighborhood.
    visualization_msgs::Marker state_pos_msg;
    state_pos_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    state_pos_msg.ns = "/mob/ellis_planner/error_nbhds/pos";
    state_pos_msg.id = i;
    state_pos_msg.type = visualization_msgs::Marker::SPHERE;
    state_pos_msg.pose.position.x = nbhd.x_;
    state_pos_msg.pose.position.y = nbhd.y_;
    state_pos_msg.pose.position.z = -0.674614;  // TODO(eratner) Fix this
    state_pos_msg.pose.orientation.x = 0;
    state_pos_msg.pose.orientation.y = 0;
    state_pos_msg.pose.orientation.z = 0;
    state_pos_msg.pose.orientation.w = 1;
    state_pos_msg.color.r = 0;
    state_pos_msg.color.g = 0.5;
    state_pos_msg.color.b = 0.5;
    state_pos_msg.color.a = 0.5;
    state_pos_msg.scale.x = env_.GetExecutionErrorNeighborhoodParameters().state_radius_pos_;
    state_pos_msg.scale.y = env_.GetExecutionErrorNeighborhoodParameters().state_radius_pos_;
    state_pos_msg.scale.z = env_.GetExecutionErrorNeighborhoodParameters().state_radius_pos_;
    vis_pub_.publish(state_pos_msg);

    visualization_msgs::Marker action_msg;
    action_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    action_msg.ns = "/mob/ellis_planner/error_nbhds/action";
    action_msg.id = i;
    action_msg.type = visualization_msgs::Marker::ARROW;
    action_msg.pose.position.x = nbhd.x_;
    action_msg.pose.position.y = nbhd.y_;
    action_msg.pose.position.z = -0.674614;  // TODO(eratner) Fix this
    tf2::Quaternion orien;
    if (std::abs(nbhd.action_dir_yaw_) > 1e-6) {
      if (nbhd.action_dir_yaw_ > 0.0)
        orien.setRPY(-M_PI_2, 0.0, 0.0);
      else
        orien.setRPY(M_PI_2, 0.0, 0.0);
    } else {
      double angle = std::atan2(nbhd.action_dir_y_, nbhd.action_dir_x_);
      orien.setRPY(0.0, 0.0, angle);
    }
    action_msg.pose.orientation.x = orien.x();
    action_msg.pose.orientation.y = orien.y();
    action_msg.pose.orientation.z = orien.z();
    action_msg.pose.orientation.w = orien.w();
    action_msg.color.r = 0;
    action_msg.color.g = 0.4;
    action_msg.color.b = 0.6;
    action_msg.color.a = 0.5;
    action_msg.scale.x = 1.25 * env_.GetExecutionErrorNeighborhoodParameters().state_radius_pos_;
    action_msg.scale.y = 0.05;
    action_msg.scale.z = 0.05;
    vis_pub_.publish(action_msg);
  }
}

bool PlannerInterface::ClearExecutionErrors(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  env_.ClearExecutionErrorNeighborhoods();
  return true;
}

void PlannerInterface::PublishPathMarkers(const std::vector<ellis_planner::State::Ptr>& path) {
  for (int i = 0; i < path.size(); ++i) {
    auto state = path[i];
    auto robot = env_.GetRobotCollisionObject();
    robot->SetX(state->GetX());
    robot->SetY(state->GetY());
    robot->SetYaw(state->GetYaw());
    auto msg = robot->GetMarker();
    msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    msg.ns = "/mob/ellis_planner/path";
    msg.id = i;
    vis_pub_.publish(msg);
  }
}

void PlannerInterface::DeletePathMarkers() {
  for (int i = 0; i < 200; ++i) {  // TODO(eratner) This is hacky...
    visualization_msgs::Marker msg;
    msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    msg.ns = "/mob/ellis_planner/path";
    msg.id = i;
    msg.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(msg);
  }
}

}  // namespace ellis_planner

PLUGINLIB_EXPORT_CLASS(ellis_planner::PlannerInterface, nodelet::Nodelet);
