// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_PLANNER_INTERFACE_H_
#define ELLIS_PLANNER_PLANNER_INTERFACE_H_

#include <ros/ros.h>
#include <choreographer/planner.h>
#include <tf2_ros/transform_listener.h>
#include <ff_util/ff_names.h>
#include <visualization_msgs/Marker.h>
#include <ellis_planner/polynomial_trajectory.h>
#include <ellis_planner/environment.h>
#include <ellis_planner/search.h>
#include <ellis_planner/ReportExecutionError.h>
#include <ellis_planner/AddObstacle.h>
#include <ellis_planner/PlanningInfo.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <vector>
#include <cmath>

namespace ellis_planner {

// TODO(eratner) Move somewhere else...
template <typename MessageType>
class ScopedPublish {
 public:
  explicit ScopedPublish(ros::Publisher* pub) : pub_(pub) {}

  ~ScopedPublish() { pub_->publish(msg_); }

  ros::Publisher* pub_;
  MessageType msg_;
};

class PlannerInterface : public planner::PlannerImplementation {
 public:
  typedef std::array<double, kStateDim> Waypoint;

  PlannerInterface();

  ~PlannerInterface();

 protected:
  bool InitializePlanner(ros::NodeHandle* nh);

  bool ReconfigureCallback(dynamic_reconfigure::Config& config);

  void PlanCallback(const ff_msgs::PlanGoal& goal);

  PolynomialTrajectory<kStateDim> ToTrajectory(const std::vector<Waypoint>& waypoints, double start_time_sec);

  double GetTimeBetweenWaypoints(const Waypoint& first_waypoint, const Waypoint& second_waypoint) const;

  bool GetPose(double& x, double& y, double& z, double& yaw);

  void PublishPoseMarker(double x, double y, double z, double yaw, const std::string& name = "pose",
                         bool show_name = true, double r = 1.0, double g = 0.0, double b = 0.0, double a = 0.75);

  bool AddObstacle(AddObstacle::Request& req, AddObstacle::Response& res);

  bool ClearObstacles(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // bool ReportExecutionError(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool ReportExecutionError(ReportExecutionError::Request& req, ReportExecutionError::Response& res);

  void PublishExecutionErrorNeighborhoodMarkers();

  bool ClearExecutionErrors(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  void PublishObstacleMarkers();

  void PublishPathMarkers(const std::vector<ellis_planner::State::Ptr>& path);

  void DeletePathMarkers();

  void PublishWeightedPenalties(double min_x, double max_x, double min_y, double max_y);

  void PublishWeightedPenaltyHeatmap(double min_x, double max_x, double min_y, double max_y);

  ff_util::ConfigServer cfg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher vis_pub_;
  ros::Publisher planning_info_pub_;

  ros::ServiceServer add_obstacle_srv_;
  ros::ServiceServer clear_obstacles_srv_;
  ros::ServiceServer report_execution_error_srv_;
  ros::ServiceServer clear_execution_errors_srv_;

  double nominal_lin_vel_;
  double nominal_ang_vel_;

  // Planning.
  Environment env_;
  Search search_;

  std::vector<ff_msgs::ControlState> last_trajectory_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_PLANNER_INTERFACE_H_
