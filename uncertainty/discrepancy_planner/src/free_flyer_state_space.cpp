#include <discrepancy_planner/free_flyer_state_space.h>

namespace discrepancy_planner {

FreeFlyerStateSpace::Discretizer::Discretizer(
  const std::array<double, StateDim>& disc)
    : disc_(disc) {}

int FreeFlyerStateSpace::Discretizer::Discretize(double value,
                                                 unsigned int index) const {
  return static_cast<int>(value / disc_[index]);
}

double FreeFlyerStateSpace::Discretizer::Undiscretize(
  int value, unsigned int index) const {
  return static_cast<double>(value) * disc_[index];
}

const std::array<double, FreeFlyerStateSpace::StateDim>&
FreeFlyerStateSpace::Discretizer::GetDiscretization() const {
  return disc_;
}

void FreeFlyerStateSpace::Discretizer::SetDiscretization(
  const std::array<double, StateDim>& disc) {
  disc_ = disc;
}

std::string FreeFlyerStateSpace::DimensionalityToStr(Dimensionality dim) {
  switch (dim) {
    case LOW_D:
      return "low_d";
    case HIGH_D:
      return "high_d";
    default:
      break;
  }
  return "";
}

FreeFlyerStateSpace::MotionPrimitive::MotionPrimitive(
  int id, CostType cost, Dimensionality dim, int vel_x, int vel_y, int vel_z,
  int vel_yaw, int vel_prox_angle, int vel_dist_angle)
    : id_(id),
      cost_(cost),
      dim_(dim),
      vel_x_(vel_x),
      vel_y_(vel_y),
      vel_z_(vel_z),
      vel_yaw_(vel_yaw),
      vel_prox_angle_(vel_prox_angle),
      vel_dist_angle_(vel_dist_angle) {}

std::string FreeFlyerStateSpace::VariableIndexToStr(VariableIndex index) {
  switch (index) {
    case X:
      return "X";
    case Y:
      return "Y";
    case Z:
      return "Z";
    case YAW:
      return "YAW";
    case PROX_ANGLE:
      return "PROX_ANGLE";
    case DIST_ANGLE:
      return "DIST_ANGLE";
    default:
      break;
  }
  return "";
}

bool FreeFlyerStateSpace::DiscrepancyNeighborhood::Contains(
  const Discretizer& discretizer, const State* state,
  ActionIndex action) const {
  if (action_ != action) return false;

  double x = discretizer.Undiscretize(state->GetVariables()[X], X);
  double y = discretizer.Undiscretize(state->GetVariables()[Y], Y);
  double z = discretizer.Undiscretize(state->GetVariables()[Z], Z);
  double dist =
    std::sqrt(std::pow(x - state_.x_, 2) + std::pow(y - state_.y_, 2) +
              std::pow(z - state_.z_, 2));
  if (dist > radius_.pos_) return false;

  double yaw = discretizer.Undiscretize(state->GetVariables()[YAW], YAW);

  if (std::abs(angles::shortest_angular_distance(yaw, state_.yaw_)) >
      radius_.orien_)
    return false;

  double prox_angle =
    discretizer.Undiscretize(state->GetVariables()[PROX_ANGLE], PROX_ANGLE);
  if (std::abs(prox_angle - state_.prox_angle_) > radius_.prox_angle_)
    return false;

  double dist_angle =
    discretizer.Undiscretize(state->GetVariables()[DIST_ANGLE], DIST_ANGLE);
  if (std::abs(dist_angle - state_.dist_angle_) > radius_.dist_angle_)
    return false;

  return true;
}

FreeFlyerStateSpace::FreeFlyerStateSpace(
  const Eigen::Matrix<double, 4, 4>& goal_in_world, double goal_dist_thresh,
  double goal_angle_thresh, const Discretizer& discretizer)
    : discretizer_(discretizer),
      goal_dist_thresh_(goal_dist_thresh),
      goal_angle_thresh_(goal_angle_thresh),
      variable_lower_bound_(
        DefaultValueArray<int, StateDim>(std::numeric_limits<int>::min())),
      variable_upper_bound_(
        DefaultValueArray<int, StateDim>(std::numeric_limits<int>::max())) {
  // TODO All of the below should be loaded in via parameters.
  // NOTE Adding margin.
  const double robot_collision_margin = 0.1;
  robot_collision_body_ =
    Box(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
        0.32 + robot_collision_margin, 0.32 + robot_collision_margin,
        0.32 + robot_collision_margin);

  // Set the goal pose in the world frame.
  goal_pos_in_world_ = goal_in_world.block<3, 1>(0, 3);
  Eigen::Quaterniond goal_in_world_rot(goal_in_world.block<3, 3>(0, 0));
  auto euler = goal_in_world_rot.toRotationMatrix().eulerAngles(2, 1, 0);
  goal_yaw_ = euler[0];
  goal_pitch_ = euler[1];
  goal_roll_ = euler[2];
}

FreeFlyerStateSpace::~FreeFlyerStateSpace() {}

const FreeFlyerStateSpace::Discretizer& FreeFlyerStateSpace::GetDiscretizer()
  const {
  return discretizer_;
}

double FreeFlyerStateSpace::GetGoalDistThresh() const {
  return goal_dist_thresh_;
}

void FreeFlyerStateSpace::SetGoalDistThresh(double thresh) {
  goal_dist_thresh_ = thresh;
}

double FreeFlyerStateSpace::GetGoalAngleThresh() const {
  return goal_angle_thresh_;
}

void FreeFlyerStateSpace::SetGoalAngleThresh(double thresh) {
  goal_angle_thresh_ = thresh;
}

void FreeFlyerStateSpace::SetGoalPose(
  const Eigen::Matrix<double, 4, 4>& goal_in_world) {
  goal_pos_in_world_ = goal_in_world.block<3, 1>(0, 3);
  Eigen::Quaterniond goal_in_world_rot(goal_in_world.block<3, 3>(0, 0));
  auto euler = goal_in_world_rot.toRotationMatrix().eulerAngles(2, 1, 0);
  goal_yaw_ = euler[0];
  goal_pitch_ = euler[1];
  goal_roll_ = euler[2];
}

void FreeFlyerStateSpace::GetGoalPose(double& x, double& y, double& z,
                                      double& roll, double& pitch,
                                      double& yaw) const {
  x = goal_pos_in_world_(0);
  y = goal_pos_in_world_(1);
  z = goal_pos_in_world_(2);
  roll = goal_roll_;
  pitch = goal_pitch_;
  yaw = goal_yaw_;
}

bool FreeFlyerStateSpace::IsGoalPose(double x, double y, double z, double roll,
                                     double pitch, double yaw,
                                     double prox_angle,
                                     double dist_angle) const {
  // Check tolerance on the distance from the goal.
  Eigen::Vector3d pos_in_world(x, y, z);
  if ((goal_pos_in_world_ - pos_in_world).norm() > goal_dist_thresh_)
    return false;

  // TODO Should we be more careful about checking the angles?
  // Check tolerance on angles from goal.
  if (std::abs(goal_roll_ - roll) > goal_angle_thresh_ ||
      std::abs(goal_pitch_ - pitch) > goal_angle_thresh_ ||
      std::abs(goal_yaw_ - yaw) > goal_angle_thresh_)
    return false;

  return true;
}

bool FreeFlyerStateSpace::IsGoal(const Variables& variables) const {
  double x, y, z, roll, pitch, yaw, prox_angle, dist_angle;
  GetPose(variables, x, y, z, roll, pitch, yaw, prox_angle, dist_angle);
  return IsGoalPose(x, y, z, roll, pitch, yaw, prox_angle, dist_angle);
}

void FreeFlyerStateSpace::GetPose(const Variables& variables, double& x,
                                  double& y, double& z, double& roll,
                                  double& pitch, double& yaw,
                                  double& prox_angle,
                                  double& dist_angle) const {
  x = discretizer_.Undiscretize(variables[X], X);
  y = discretizer_.Undiscretize(variables[Y], Y);
  z = discretizer_.Undiscretize(variables[Z], Z);
  roll = 0;
  pitch = 0;
  yaw = discretizer_.Undiscretize(variables[YAW], YAW);
  prox_angle = discretizer_.Undiscretize(variables[PROX_ANGLE], PROX_ANGLE);
  dist_angle = discretizer_.Undiscretize(variables[DIST_ANGLE], DIST_ANGLE);
}

FreeFlyerStateSpace::State* FreeFlyerStateSpace::GetState(
  double x, double y, double z, double roll, double pitch, double yaw,
  double prox_angle, double dist_angle) {
  Variables variables;
  variables[X] = discretizer_.Discretize(x, X);
  variables[Y] = discretizer_.Discretize(y, Y);
  variables[Z] = discretizer_.Discretize(z, Z);
  variables[YAW] = discretizer_.Discretize(yaw, YAW);
  variables[PROX_ANGLE] = discretizer_.Discretize(prox_angle, PROX_ANGLE);
  variables[DIST_ANGLE] = discretizer_.Discretize(dist_angle, DIST_ANGLE);

  return StateSpace<StateDim>::GetState(variables);
}

std::vector<FreeFlyerStateSpace::ActionIndex> FreeFlyerStateSpace::GetActions(
  const State* state) const {
  std::vector<ActionIndex> actions;

  // Add actions depending on the dimensionality of the state.
  // TODO Implement adaptive dimensionality
  // auto dim = GetDimensionality(state->GetVariables());
  Dimensionality dim = HIGH_D;
  for (const auto& m : motion_primitives_) {
    if (m.dim_ <= dim) actions.push_back(m.id_);
  }

  return actions;
}

std::tuple<std::vector<FreeFlyerStateSpace::State*>,
           std::vector<FreeFlyerStateSpace::CostType>,
           std::vector<FreeFlyerStateSpace::ProbabilityType>>
FreeFlyerStateSpace::GetSucc(const State* state, ActionIndex action) {
  std::vector<State*> succs;
  std::vector<CostType> costs;
  std::vector<ProbabilityType> probs;

  if (action >= 0 && action < motion_primitives_.size()) {
    const auto& primitive = motion_primitives_[action];

    Variables succ_vars = state->GetVariables();

    succ_vars[X] += primitive.vel_x_;
    succ_vars[Y] += primitive.vel_y_;
    succ_vars[Z] += primitive.vel_z_;
    succ_vars[YAW] += primitive.vel_yaw_;
    succ_vars[PROX_ANGLE] += primitive.vel_prox_angle_;
    succ_vars[DIST_ANGLE] += primitive.vel_dist_angle_;

    // Check the bounds on the successor's state variables.
    for (int i = X; i <= DIST_ANGLE; ++i) {
      if (!IsInBounds(static_cast<VariableIndex>(i), succ_vars[i])) {
        // Out of bounds!
        return std::make_tuple(succs, costs, probs);
      }
    }

    // Collision check the transition.
    if (!InCollisionBetween(state->GetVariables(), succ_vars)) {
      succs = {StateSpace<StateDim>::GetState(succ_vars)};
      double cost = primitive.cost_ + GetPenalty(state, action);
      costs = {cost};
      probs = {1.0};
    }
  } else {
    ROS_ERROR_STREAM("[FreeFlyerStateSpace::GetSucc] Invalid action " << action
                                                                      << "!");
  }

  return std::make_tuple(succs, costs, probs);
}

bool FreeFlyerStateSpace::InCollisionBetween(const Variables& state_variables,
                                             const Variables& succ_variables) {
  // High-D collision checking.
  double state_x, state_y, state_z, state_roll, state_pitch, state_yaw,
    state_prox_angle, state_dist_angle;
  GetPose(state_variables, state_x, state_y, state_z, state_roll, state_pitch,
          state_yaw, state_prox_angle, state_dist_angle);

  double succ_x, succ_y, succ_z, succ_roll, succ_pitch, succ_yaw,
    succ_prox_angle, succ_dist_angle;
  GetPose(succ_variables, succ_x, succ_y, succ_z, succ_roll, succ_pitch,
          succ_yaw, succ_prox_angle, succ_dist_angle);

  const int num_checks = 2;  // TODO Make a parameter
  for (int i = 0; i <= num_checks; ++i) {
    // Does NOT check the successor state.
    // double p = (i + 1.0) / static_cast<double>(num_checks + 2);

    // Does check the successor state.
    double p = (i + 1.0) / static_cast<double>(num_checks + 1);

    // Linearly interpolate along the transition.
    double sample_x = state_x + p * (succ_x - state_x);
    double sample_y = state_y + p * (succ_y - state_y);
    double sample_z = state_z + p * (succ_z - state_z);
    double sample_roll = state_roll + p * (succ_roll - state_roll);
    double sample_pitch = state_pitch + p * (succ_pitch - state_pitch);
    double sample_yaw = state_yaw + p * (succ_yaw - state_yaw);
    double sample_prox_angle =
      state_prox_angle + p * (succ_prox_angle - state_prox_angle);
    double sample_dist_angle =
      state_dist_angle + p * (succ_dist_angle - state_dist_angle);

    const auto& robot_body = GetRobotCollisionBody(
      sample_x, sample_y, sample_z, sample_roll, sample_pitch, sample_yaw);

    // Check for collisions between the robot and other bodies
    // (non-moveable).
    for (const auto& world_body : world_collision_bodies_) {
      if (world_body.Intersects(robot_body)) {
        // In collision!
        return true;
      }
    }
  }

  return false;
}

const Box& FreeFlyerStateSpace::GetRobotCollisionBody(double x, double y,
                                                      double z, double roll,
                                                      double pitch,
                                                      double yaw) {
  robot_collision_body_.SetPosition(Eigen::Vector3d(x, y, z));
  robot_collision_body_.SetOrientationEuler(roll, pitch, yaw);
  return robot_collision_body_;
}

const std::vector<Box>& FreeFlyerStateSpace::GetWorldCollisionBodies() const {
  return world_collision_bodies_;
}

void FreeFlyerStateSpace::AddWorldCollisionBody(const Box& body) {
  world_collision_bodies_.push_back(body);
}

void FreeFlyerStateSpace::SetVariableLowerBound(unsigned int index,
                                                double bound) {
  if (index >= StateDim) {
    ROS_ERROR_STREAM("[SetVariableLowerBound] Error: index "
                     << index << " is not valid!");
    return;
  }
  variable_lower_bound_[index] = discretizer_.Discretize(bound, index);
}

void FreeFlyerStateSpace::SetVariableUpperBound(unsigned int index,
                                                double bound) {
  if (index >= StateDim) {
    ROS_ERROR_STREAM("[SetVariableUpperBound] Error: index "
                     << index << " is not valid!");
    return;
  }
  variable_upper_bound_[index] = discretizer_.Discretize(bound, index);
}

bool FreeFlyerStateSpace::IsInBounds(unsigned int index, int value) {
  if (index >= StateDim) {
    ROS_ERROR_STREAM("[IsInBounds] Error: index " << index << " is not valid!");
    return false;
  }
  return (variable_lower_bound_[index] <= value &&
          value <= variable_upper_bound_[index]);
}

const std::vector<FreeFlyerStateSpace::MotionPrimitive>&
FreeFlyerStateSpace::GetMotionPrimitives() const {
  return motion_primitives_;
}

void FreeFlyerStateSpace::SetMotionPrimitives(
  const std::vector<FreeFlyerStateSpace::MotionPrimitive>& primitives) {
  motion_primitives_ = primitives;
}

bool FreeFlyerStateSpace::LoadMotionPrimitives(
  const XmlRpc::XmlRpcValue& param) {
  motion_primitives_.clear();
  int id = 0;
  for (int i = 0; i < param.size(); ++i) {
    const auto& prim = param[i];

    MotionPrimitive m;
    m.id_ = id;

    if (!prim.hasMember("cost")) {
      ROS_WARN_STREAM("Warning: motion primitive "
                      << i << " has no cost specified!");
      continue;
    }

    if (prim["cost"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      m.cost_ = static_cast<int>(prim["cost"]);
    else if (prim["cost"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      m.cost_ = static_cast<double>(prim["cost"]);
    else {
      ROS_WARN_STREAM("Warning: motion primitive "
                      << i << " has invalid cost type!");
      continue;
    }

    if (!prim.hasMember("dim")) {
      ROS_WARN_STREAM("Warning: motion primitive "
                      << i << " has no dimensionality specified!");
      continue;
    }

    if (prim["dim"].getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::string dim_str = static_cast<std::string>(prim["dim"]);
      if (dim_str == DimensionalityToStr(LOW_D))
        m.dim_ = LOW_D;
      else if (dim_str == DimensionalityToStr(HIGH_D))
        m.dim_ = HIGH_D;
      else {
        ROS_WARN_STREAM("Warning: motion primitive "
                        << i << " has invalid dim \"" << dim_str << "\"");
        continue;
      }
    } else {
      ROS_WARN_STREAM("Warning: motion primitive " << i
                                                   << " has invalid dim type!");
      continue;
    }

    for (unsigned int k = 0; k <= DIST_ANGLE; ++k) {
      std::string name =
        "vel_" + VariableIndexToStr(static_cast<VariableIndex>(k));
      std::transform(name.begin(), name.end(), name.begin(),
                     [](unsigned char c) { return std::tolower(c); });

      int input = -1;
      if (prim.hasMember(name)) {
        if (prim[name].getType() == XmlRpc::XmlRpcValue::TypeInt)
          input = static_cast<int>(prim[name]);
        else if (prim[name].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          input = static_cast<double>(prim[name]);
        else {
          ROS_ERROR_STREAM("Motion primitive " << i << " has invalid " << name
                                               << " type!");
          return false;
        }
      } else
        continue;

      switch (k) {
        case X:
          m.vel_x_ = input;
          break;
        case Y:
          m.vel_y_ = input;
          break;
        case Z:
          m.vel_z_ = input;
          break;
        case YAW:
          m.vel_yaw_ = input;
          break;
        case PROX_ANGLE:
          m.vel_prox_angle_ = input;
          break;
        case DIST_ANGLE:
          m.vel_dist_angle_ = input;
          break;
        default:
          ROS_WARN("Something went wrong!");
          break;
      }
    }

    ROS_INFO_STREAM("  vel_x: " << m.vel_x_ << ", vel_y: " << m.vel_y_
                                << ", vel_z: " << m.vel_z_
                                << ", vel_yaw: " << m.vel_yaw_
                                << ", vel_prox_angle: " << m.vel_prox_angle_
                                << ", vel_dist_angle: " << m.vel_dist_angle_);

    motion_primitives_.push_back(m);
    id++;
  }
  ROS_INFO_STREAM("Successfully loaded " << motion_primitives_.size()
                                         << " motion primitives");
  return true;
}

FreeFlyerStateSpace::CostType FreeFlyerStateSpace::GetPenalty(
  const State* state, ActionIndex action) const {
  auto it =
    std::find_if(discrepancies_.begin(), discrepancies_.end(),
                 [&](const DiscrepancyNeighborhood& discrepancy) {
                   return discrepancy.Contains(discretizer_, state, action);
                 });
  if (it != discrepancies_.end()) return it->penalty_;

  return 0;
}

void FreeFlyerStateSpace::AddDiscrepancy(
  const DiscrepancyNeighborhood& discrepancy) {
  // TODO Add high-D region around discrepancy.
  discrepancies_.push_back(discrepancy);
}

const std::vector<FreeFlyerStateSpace::DiscrepancyNeighborhood>&
FreeFlyerStateSpace::GetDiscrepancies() const {
  return discrepancies_;
}

void FreeFlyerStateSpace::ClearDiscrepancies() { discrepancies_.clear(); }

}  // namespace discrepancy_planner
