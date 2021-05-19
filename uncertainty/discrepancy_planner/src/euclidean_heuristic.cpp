#include <discrepancy_planner/euclidean_heuristic.h>

namespace discrepancy_planner {

EuclideanHeuristic::EuclideanHeuristic(FreeFlyerStateSpace* state_space,
                                       double cost_per_meter)
    : state_space_(state_space), cost_per_meter_(cost_per_meter) {}

EuclideanHeuristic::CostType EuclideanHeuristic::Value(State* state) {
  // Get the robot's position at the specified state.
  double x, y, z, unused;
  state_space_->GetPose(state->GetVariables(), x, y, z, unused, unused, unused,
                        unused, unused);

  // Get the robot's goal position.
  double goal_x, goal_y, goal_z;
  state_space_->GetGoalPose(goal_x, goal_y, goal_z, unused, unused, unused);

  // Compute the distance to the goal.
  double dist_to_goal =
    std::sqrt(std::pow(goal_x - x, 2) + std::pow(goal_y - y, 2) +
              std::pow(goal_z - z, 2));
  if (dist_to_goal <= state_space_->GetGoalDistThresh()) return 0;

  // Weight the distance by the cost per meter to estimate the cost.
  return (cost_per_meter_ * dist_to_goal);
}

}  // namespace discrepancy_planner
