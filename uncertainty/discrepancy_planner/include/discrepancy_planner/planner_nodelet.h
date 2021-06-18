#ifndef DISCREPANCY_PLANNER_PLANNER_NODELET_H
#define DISCREPANCY_PLANNER_PLANNER_NODELET_H

#include <ros/ros.h>
#include <choreographer/planner.h>
#include <tf2_ros/transform_listener.h>
#include <ff_util/ff_names.h>
#include <astrobee_search_based_planning/euclidean_heuristic.h>
#include <astrobee_search_based_planning/polynomial_trajectory.h>
#include <astrobee_search_based_planning/rtaa.h>
#include <astrobee_search_based_planning/state_space.h>
#include <astrobee_search_based_planning/box.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
using astrobee_search_based_planning::Box;
using astrobee_search_based_planning::EuclideanHeuristic;
using astrobee_search_based_planning::PolynomialTrajectory;
using astrobee_search_based_planning::RTAA;

namespace discrepancy_planner {

// Trajectory waypoint: (x, y, z, yaw, prox. angle, dist. angle)
constexpr unsigned int kTrajectoryDim = 6;

class PlannerNodelet : public planner::PlannerImplementation {
 public:
  typedef PolynomialTrajectory<kTrajectoryDim> Trajectory;
  typedef astrobee_search_based_planning::StateSpace<0> StateSpace;

  enum TrajectoryIndex {
    TRAJECTORY_X = 0,
    TRAJECTORY_Y,
    TRAJECTORY_Z,
    TRAJECTORY_YAW,
    TRAJECTORY_PROX_ANGLE,
    TRAJECTORY_DIST_ANGLE
  };

  PlannerNodelet();

  ~PlannerNodelet();

 protected:
  bool InitializePlanner(ros::NodeHandle* nh);

  bool ParseDoubleParameter(const XmlRpc::XmlRpcValue& value,
                            double& value_out) const;

  bool ReconfigureCallback(dynamic_reconfigure::Config& config);

  void PlanCallback(ff_msgs::PlanGoal const& goal);

  void MapPoseToSearchGraph(double x_in, double y_in, double z_in,
                            double yaw_in, double& x_out, double& y_out,
                            double& z_out, double& yaw_out) const;

  double MapToSearchGraph(double value, StateSpace::VariableIndex variable,
                          int units_per_transition) const;

  Trajectory PathToTrajectory(const std::vector<StateSpace::State*>& path,
                              double start_x, double start_y, double start_z,
                              double start_yaw, double start_prox_angle,
                              double start_dist_angle,
                              double start_time_sec = 0);

  double GetTimeBetweenWaypoints(
    const std::array<double, kTrajectoryDim>& first_waypoint,
    const std::array<double, kTrajectoryDim>& second_waypoint) const;

  bool AddDiscrepancy(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);

  void PublishGoalMarker();

  void PublishPathMarker(const std::vector<StateSpace::State*>& path);

  void PublishDiscrepancyMarkers();

  ff_util::ConfigServer cfg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher vis_pub_;

  ros::ServiceServer add_discrepancy_srv_;

  double nominal_lin_vel_;
  double nominal_ang_vel_;
  double nominal_joint_vel_;

  StateSpace* state_space_;
  ellis_util::search::Heuristic<StateSpace::Dim>* heuristic_;
  RTAA<StateSpace::Dim>* search_;
  Trajectory last_trajectory_;
  std::vector<StateSpace::ActionIndex> last_actions_;
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_PLANNER_NODELET_H
