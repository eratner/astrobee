#ifndef DISCREPANCY_PLANNER_PLANNER_NODELET_H
#define DISCREPANCY_PLANNER_PLANNER_NODELET_H

#include <ros/ros.h>
#include <choreographer/planner.h>
#include <tf2_ros/transform_listener.h>
#include <ff_util/ff_names.h>
#include <discrepancy_planner/util.h>
#include <discrepancy_planner/free_flyer_state_space.h>
#include <discrepancy_planner/euclidean_heuristic.h>
#include <discrepancy_planner/rtaa.h>
#include <discrepancy_planner/polynomial_trajectory.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>

namespace discrepancy_planner {

constexpr unsigned int kTrajectoryDim = 6;

class PlannerNodelet : public planner::PlannerImplementation {
 public:
  PlannerNodelet();

  ~PlannerNodelet();

 protected:
  bool InitializePlanner(ros::NodeHandle* nh);

  bool ReconfigureCallback(dynamic_reconfigure::Config& config);

  void PlanCallback(ff_msgs::PlanGoal const& goal);

  void MapPoseToSearchGraph(double x_in, double y_in, double z_in,
                            double yaw_in, double& x_out, double& y_out,
                            double& z_out, double& yaw_out) const;

  double MapToSearchGraph(double value,
                          FreeFlyerStateSpace::VariableIndex variable,
                          int units_per_transition) const;

  PolynomialTrajectory<kTrajectoryDim> PathToTrajectory(
    const std::vector<FreeFlyerStateSpace::State*>& path, double start_x,
    double start_y, double start_z, double start_yaw, double start_prox_angle,
    double start_dist_angle, double start_time_sec = 0);

  double GetTimeBetweenWaypoints(
    const std::array<double, kTrajectoryDim>& first_waypoint,
    const std::array<double, kTrajectoryDim>& second_waypoint) const;

  bool AddDiscrepancy(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);

  void PublishGoalMarker();

  void PublishPathMarker(const std::vector<FreeFlyerStateSpace::State*>& path);

  void PublishDiscrepancyMarkers();

  ff_util::ConfigServer cfg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher vis_pub_;

  ros::ServiceServer add_discrepancy_srv_;

  double nominal_lin_vel_;
  double nominal_ang_vel_;
  double nominal_joint_vel_;

  FreeFlyerStateSpace* state_space_;
  ellis_util::search::Heuristic<FreeFlyerStateSpace::StateDim>* heuristic_;
  RTAA<FreeFlyerStateSpace::StateDim>* search_;
  PolynomialTrajectory<kTrajectoryDim> last_trajectory_;
  std::vector<FreeFlyerStateSpace::ActionIndex> last_actions_;
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_PLANNER_NODELET_H
