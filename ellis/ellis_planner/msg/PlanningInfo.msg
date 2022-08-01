# Copyright 2022 Ellis Ratner (eratner@berkeley.edu)

# Start and goal states.
geometry_msgs/Pose start_state
geometry_msgs/Pose goal_state

# Environment bounds.
geometry_msgs/Point bounds_min
geometry_msgs/Point bounds_max

# TODO(eratner) Known obstacles in the environment.

# Planner performance info.
float64 planning_time_sec
int32 num_expansions

# Planner solution info.
bool success
geometry_msgs/Pose[] path
float64 path_cost

# Planner action set info.
geometry_msgs/Twist[] actions
string[] action_names
float64[] action_costs

# Info about discrepancies between execution and planning model.
geometry_msgs/Pose[] discrepancy_states
geometry_msgs/Twist[] discrepancy_actions

# Info about the size of neighborhoods around discrepancies.
float64 nbhd_state_radius_pos
float64 nbhd_state_radius_yaw
float64 nbhd_action_radius_pos
float64 nbhd_action_radius_yaw
float64 nbhd_penalty

