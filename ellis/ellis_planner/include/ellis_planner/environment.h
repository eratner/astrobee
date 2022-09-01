// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_ENVIRONMENT_H_
#define ELLIS_PLANNER_ENVIRONMENT_H_

#include <ellis_planner/state.h>
#include <ellis_planner/rectangle_collision_object.h>
#include <ellis_planner/linear_dynamics.h>
#include <ellis_planner/normal_dist.h>
#include <ellis_planner/clock.h>
#include <ellis_planner/ReportExecutionError.h>
#include <boost/functional/hash.hpp>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <string>
#include <cmath>
// #include <cfenv> // TODO(eratner) Prohibited header?

namespace ellis_planner {

double AngularDist(double from, double to);

class Environment {
 public:
  struct Action {
    explicit Action(const std::string& name = "nop", double change_in_x = 0.0, double change_in_y = 0.0,
                    double change_in_yaw = 0.0, double cost = 1.0);

    std::string name_;
    double change_in_x_;
    double change_in_y_;
    double change_in_yaw_;
    double cost_;
  };

  struct ExecutionErrorNeighborhoodParameters {
    explicit ExecutionErrorNeighborhoodParameters(double state_radius_pos = 0.2, double state_radius_yaw = 0.5,
                                                  double action_radius_pos = 0.2, double penalty = 0.0);

    double state_radius_pos_;
    double state_radius_yaw_;
    double action_radius_pos_;
    double penalty_;
  };

  struct ExecutionErrorNeighborhood {
    explicit ExecutionErrorNeighborhood(double x = 0.0, double y = 0.0, double yaw = 0.0, double action_dir_x = 0.0,
                                        double action_dir_y = 0.0, double action_dir_yaw = 0.0);

    explicit ExecutionErrorNeighborhood(const ReportExecutionError::Request& req);

    bool Contains(const State::Ptr state, const Action& action,
                  const ExecutionErrorNeighborhoodParameters& params) const;

    double x_;
    double y_;
    double yaw_;
    double action_dir_x_;
    double action_dir_y_;
    double action_dir_yaw_;
  };

  Environment();

  ~Environment();

  void Clear();

  State::Ptr GetState(unsigned int state_id);

  State::Ptr GetState(double x, double y, double yaw);

  bool IsGoal(const State::Ptr state) const;

  void SetGoal(double x, double y, double yaw);

  void SetGoalPosTol(double pos_tol);

  void SetGoalAngTol(double ang_tol);

  void SetBounds(double min_x, double max_x, double min_y, double max_y);

  void GetBounds(double& min_x, double& max_x, double& min_y, double& max_y);

  void SetActions(const std::vector<Action>& actions);

  const std::vector<Action>& GetActions() const;

  RectangleCollisionObject::Ptr GetRobotCollisionObject();

  void AddCollisionObject(CollisionObject::Ptr col);

  const std::vector<CollisionObject::Ptr>& GetCollisionObjects();

  void ClearCollisionObjects();

  // Returns a tuple of the outcome state and the cost of taking the action.
  std::tuple<State::Ptr, double> GetOutcome(const State::Ptr state, const Action& action);

  // TODO(eratner) Might be better to have a separate Heuristic class
  double GetHeuristicCostToGoal(const State::Ptr state) const;

  void SetExecutionErrorNeighborhoodParameters(const ExecutionErrorNeighborhoodParameters& params);

  const ExecutionErrorNeighborhoodParameters& GetExecutionErrorNeighborhoodParameters() const;

  void AddExecutionErrorNeighborhood(const ExecutionErrorNeighborhood& n);

  void ClearExecutionErrorNeighborhoods();

  const std::vector<ExecutionErrorNeighborhood>& GetExecutionErrorNeighborhoods() const;

  double GetPenalty(const State::Ptr state, const Action& action) const;

  double GetWeightedPenalty(const State::Ptr state, const Action& action) const;

  double GetWeightedPenalty(double x, double y, double yaw, const Action& action) const;

  bool UseWeightedPenalty() const;

  void SetUseWeightedPenalty(bool use);

  struct DiscreteState {
    explicit DiscreteState(int x_disc = 0, int y_disc = 0, int yaw_disc = 0);

    bool operator==(const DiscreteState& other) const;

    struct HashFunction {
      std::size_t operator()(const DiscreteState& state) const;
    };

    int x_disc_;
    int y_disc_;
    int yaw_disc_;
  };

  std::string CollisionTestFunc(double x, double y, double yaw);

  void SetDynamics(LinearDynamics<2, 2>* dynamics);

  LinearDynamics<2, 2>* GetDynamics();

  std::vector<Eigen::Vector2d> GetOpenLoopControls(const State::Ptr state, const Action& action) const;

  double GetControlLevelPenalty(const State::Ptr state, const Action& action, unsigned int num_samples = 1000) const;

  void PredictTrajectory(const State::Ptr state, const Action& action, std::vector<Eigen::Vector2d>& pred_mean,
                         std::vector<Eigen::Matrix<double, 2, 2>>& pred_cov) const;

  double GetNominalLinVel() const;

  void SetNominalLinVel(double vel);

  bool UseControlLevelPenalty() const;

  void SetUseControlLevelPenalty(bool use);

  // TODO(eratner) Clean this up
  struct Profiling {
    void Reset();

    int GetOutcome_calls;
    double GetOutcome_time_sec;
    int GetControlLevelPenalty_calls;
    double GetControlLevelPenalty_time_sec;
    int GetControlLevelPenalty_Predict_calls;
    double GetControlLevelPenalty_Predict_time_sec;
    double GetControlLevelPenalty_sampling_time_sec;
  };
  mutable Profiling prof_;

 private:
  // Discretization parameters.
  double m_per_unit_x_;      // Meters/unit in x (discretization of x).
  double m_per_unit_y_;      // Meters/unit in y (discretization of y).
  double rad_per_unit_yaw_;  // Radians/unit in yaw (discretization of yaw).

  // Goal parameters.
  double goal_x_;
  double goal_y_;
  double goal_yaw_;

  double goal_pos_tol_;  // Goal position tolerance (meters).
  double goal_ang_tol_;  // Goal angle (yaw) tolerance (radians).

  // Boundaries of the environment.
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;

  std::vector<State::Ptr> states_;
  std::unordered_map<DiscreteState, unsigned int, DiscreteState::HashFunction> discrete_state_to_id_;

  std::vector<Action> actions_;

  RectangleCollisionObject::Ptr robot_collision_object_;
  std::vector<CollisionObject::Ptr> collision_objects_;

  // Execution errors.
  ExecutionErrorNeighborhoodParameters exec_error_params_;
  std::vector<ExecutionErrorNeighborhood> exec_error_neighborhoods_;

  bool use_weighted_penalty_;

  // Dynamics model.
  LinearDynamics<2, 2>* dynamics_;
  double nominal_lin_vel_;

  bool use_control_level_penalty_;
};

std::ostream& operator<<(std::ostream& os, const Environment::Action& action);

std::ostream& operator<<(std::ostream& os, const Environment::ExecutionErrorNeighborhoodParameters& params);

std::ostream& operator<<(std::ostream& os, const Environment::ExecutionErrorNeighborhood& nbhd);

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_ENVIRONMENT_H_
