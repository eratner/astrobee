// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/environment.h>
#include <string>
#include <vector>
#include <sstream>

namespace ellis_planner {

double AngularDist(double from, double to) {
  double diff = to - from;
  diff = fmod(fmod(diff, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  if (diff > M_PI) diff -= 2.0 * M_PI;
  return diff;
}

Environment::Action::Action(const std::string& name, double change_in_x, double change_in_y, double change_in_yaw,
                            double cost)
    : name_(name), change_in_x_(change_in_x), change_in_y_(change_in_y), change_in_yaw_(change_in_yaw), cost_(cost) {}

Environment::ExecutionErrorNeighborhoodParameters::ExecutionErrorNeighborhoodParameters(double state_radius_pos,
                                                                                        double state_radius_yaw,
                                                                                        double action_radius_pos,
                                                                                        double penalty)
    : state_radius_pos_(state_radius_pos),
      state_radius_yaw_(state_radius_yaw),
      action_radius_pos_(action_radius_pos),
      penalty_(penalty) {}

Environment::ExecutionErrorNeighborhood::ExecutionErrorNeighborhood(double x, double y, double yaw, double action_dir_x,
                                                                    double action_dir_y, double action_dir_yaw)
    : x_(x),
      y_(y),
      yaw_(yaw),
      action_dir_x_(action_dir_x),
      action_dir_y_(action_dir_y),
      action_dir_yaw_(action_dir_yaw) {}

Environment::ExecutionErrorNeighborhood::ExecutionErrorNeighborhood(const ReportExecutionError::Request& req)
    : x_(req.x),
      y_(req.y),
      yaw_(req.yaw),
      action_dir_x_(req.action_dir_x),
      action_dir_y_(req.action_dir_y),
      action_dir_yaw_(req.action_dir_yaw) {}

bool Environment::ExecutionErrorNeighborhood::Contains(const State::Ptr state, const Action& action,
                                                       const ExecutionErrorNeighborhoodParameters& params) const {
  // Check distance in state position.
  if (std::sqrt(std::pow(state->GetX() - x_, 2) + std::pow(state->GetY() - y_, 2)) > params.state_radius_pos_)
    return false;

  // Check distance in state orientation.
  if (std::abs(AngularDist(state->GetYaw(), yaw_)) > params.state_radius_yaw_) return false;

  // TODO(eratner) Define a better measure of yaw action distance
  double action_dir_yaw = 0.0;
  if (std::abs(action.change_in_yaw_) > 1e-6) {
    if (action.change_in_yaw_ < 0.0)
      action_dir_yaw = -1.0;
    else
      action_dir_yaw = 1.0;
  }
  if (std::abs(action_dir_yaw - action_dir_yaw_) > 1e-6) return false;

  // Check distance in the action (position/velocity only)-- defined as the angle between the action direction vectors.
  double n = std::sqrt(std::pow(action.change_in_x_, 2) + std::pow(action.change_in_y_, 2));
  if (n > 1e-6) {
    double action_dir_x = action.change_in_x_ / n;
    double action_dir_y = action.change_in_y_ / n;
    double angle_between_actions = std::acos(action_dir_x * action_dir_x_ + action_dir_y * action_dir_y_);
    if (std::abs(angle_between_actions) > params.action_radius_pos_) return false;
  }

  return true;
}

Environment::Environment()
    : m_per_unit_x_(1e-2),
      m_per_unit_y_(1e-2),
      rad_per_unit_yaw_(8e-2),
      goal_x_(0.0),
      goal_y_(0.0),
      goal_yaw_(0.0),
      goal_pos_tol_(1e-1),
      goal_ang_tol_(1e-1),
      use_weighted_penalty_(false) {
  // Robot is about 0.32 m x 0.32 m (TODO(eratner) shouldn't hard code this).
  robot_collision_object_ = new RectangleCollisionObject("robot", 0.0, 0.0, 0.0, 0.4, 0.4);
}

Environment::~Environment() {
  Clear();

  if (robot_collision_object_) delete robot_collision_object_;

  ClearCollisionObjects();
}

void Environment::Clear() {
  for (auto state : states_) {
    if (state) delete state;
  }

  states_.clear();
  discrete_state_to_id_.clear();
}

State::Ptr Environment::GetState(unsigned int state_id) {
  if (state_id >= states_.size()) {
    // TODO(eratner) Throw/print error
    return nullptr;
  }

  return states_[state_id];
}

State::Ptr Environment::GetState(double x, double y, double yaw) {
  DiscreteState key(static_cast<int>(x / m_per_unit_x_), static_cast<int>(y / m_per_unit_y_),
                    static_cast<int>(yaw / rad_per_unit_yaw_));
  auto it = discrete_state_to_id_.find(key);
  State::Ptr state = nullptr;
  if (it == discrete_state_to_id_.end()) {
    // State doesn't exist yet, so construct it.
    state = new State(states_.size(), key.x_disc_ * m_per_unit_x_, key.y_disc_ * m_per_unit_y_,
                      key.yaw_disc_ * rad_per_unit_yaw_);
    states_.push_back(state);
    discrete_state_to_id_[key] = state->GetId();
  } else {
    // State already exists, so retrieve it.
    state = states_[it->second];
  }

  return state;
}

bool Environment::IsGoal(const State::Ptr state) const {
  double dist_to_goal = std::sqrt(std::pow(state->GetX() - goal_x_, 2) + std::pow(state->GetY() - goal_y_, 2));
  double angle_to_goal = AngularDist(state->GetYaw(), goal_yaw_);
  return (dist_to_goal <= goal_pos_tol_ && std::abs(angle_to_goal) < goal_ang_tol_);
}

void Environment::SetGoal(double x, double y, double yaw) {
  goal_x_ = x;
  goal_y_ = y;
  goal_yaw_ = yaw;
}

void Environment::SetGoalPosTol(double pos_tol) { goal_pos_tol_ = pos_tol; }

void Environment::SetGoalAngTol(double ang_tol) { goal_ang_tol_ = ang_tol; }

void Environment::SetBounds(double min_x, double max_x, double min_y, double max_y) {
  min_x_ = min_x;
  max_x_ = max_x;
  min_y_ = min_y;
  max_y_ = max_y;
}

void Environment::GetBounds(double& min_x, double& max_x, double& min_y, double& max_y) {
  min_x = min_x_;
  max_x = max_x_;
  min_y = min_y_;
  max_y = max_y_;
}

void Environment::SetActions(const std::vector<Action>& actions) { actions_ = actions; }

const std::vector<Environment::Action>& Environment::GetActions() const { return actions_; }

RectangleCollisionObject::Ptr Environment::GetRobotCollisionObject() { return robot_collision_object_; }

void Environment::AddCollisionObject(CollisionObject::Ptr col) { collision_objects_.push_back(col); }

const std::vector<CollisionObject::Ptr>& Environment::GetCollisionObjects() { return collision_objects_; }

void Environment::ClearCollisionObjects() {
  for (auto col : collision_objects_) {
    if (col) delete col;
  }

  collision_objects_.clear();
}

std::tuple<State::Ptr, double> Environment::GetOutcome(const State::Ptr state, const Action& action) {
  double x = state->GetX() + action.change_in_x_;
  if (x < min_x_ || max_x_ < x) {
    // Out-of-bounds in x.
    return std::make_tuple(nullptr, 1000.0);
  }

  double y = state->GetY() + action.change_in_y_;
  if (y < min_y_ || max_y_ < y) {
    // Out-of-bounds in y.
    return std::make_tuple(nullptr, 1000.0);
  }

  double yaw = state->GetYaw() + action.change_in_yaw_;

  // TODO(eratner) May require more fine-grained collision checking
  robot_collision_object_->SetX(x);
  robot_collision_object_->SetY(y);
  robot_collision_object_->SetYaw(yaw);
  for (auto col : collision_objects_) {
    if (col->InCollision(robot_collision_object_)) {
      // Outcome state is in collision!
      return std::make_tuple(nullptr, 1000.0);
    }
  }

  double cost = action.cost_;
  if (UseWeightedPenalty())
    cost += GetWeightedPenalty(state, action);
  else
    cost += GetPenalty(state, action);

  auto outcome = GetState(x, y, yaw);
  return std::make_tuple(outcome, cost);
}

double Environment::GetHeuristicCostToGoal(const State::Ptr state) const {
  if (IsGoal(state)) return 0;

  // TODO(eratner) Because of goal tolerances, this could slightly overestimate the cost
  return (std::abs(goal_x_ - state->GetX()) + std::abs(goal_y_ - state->GetY()));
}

void Environment::SetExecutionErrorNeighborhoodParameters(const ExecutionErrorNeighborhoodParameters& params) {
  exec_error_params_ = params;
}

const Environment::ExecutionErrorNeighborhoodParameters& Environment::GetExecutionErrorNeighborhoodParameters() const {
  return exec_error_params_;
}

void Environment::AddExecutionErrorNeighborhood(const ExecutionErrorNeighborhood& n) {
  // TODO(eratner) Avoid adding duplicates (i.e., neighborhoods with almost identical intersections)
  exec_error_neighborhoods_.push_back(n);
}

void Environment::ClearExecutionErrorNeighborhoods() { exec_error_neighborhoods_.clear(); }

const std::vector<Environment::ExecutionErrorNeighborhood>& Environment::GetExecutionErrorNeighborhoods() const {
  return exec_error_neighborhoods_;
}

double Environment::GetPenalty(const State::Ptr state, const Action& action) const {
  double penalty = 0.0;
  for (const auto& nbhd : exec_error_neighborhoods_) {
    if (nbhd.Contains(state, action, exec_error_params_)) {
      penalty = exec_error_params_.penalty_;
      break;
    }
  }
  return penalty;
}

double Environment::GetWeightedPenalty(const State::Ptr state, const Action& action) const {
  return GetWeightedPenalty(state->GetX(), state->GetY(), state->GetYaw(), action);
}

double Environment::GetWeightedPenalty(double x, double y, double yaw, const Action& action) const {
  double penalty = 0.0;

  // TODO(eratner) Add action yaw
  typename MultivariateNormal<3>::Mat cov = MultivariateNormal<3>::Mat::Identity();
  cov(0, 0) = std::pow(exec_error_params_.state_radius_pos_, 2);
  cov(1, 1) = std::pow(exec_error_params_.state_radius_yaw_, 2);
  cov(2, 2) = std::pow(exec_error_params_.action_radius_pos_, 2);
  typename MultivariateNormal<3>::Vec mean = MultivariateNormal<3>::Vec::Zero();

  for (const auto& n : exec_error_neighborhoods_) {
    // TODO(eratner) Define a better measure of yaw action distance
    double action_dir_yaw = 0.0;
    if (std::abs(action.change_in_yaw_) > 1e-6) {
      if (action.change_in_yaw_ < 0.0)
        action_dir_yaw = -1.0;
      else
        action_dir_yaw = 1.0;
    }
    // TODO(eratner) For now, skip actions that don't match
    if (std::abs(action_dir_yaw - n.action_dir_yaw_) > 1e-6) continue;

    typename MultivariateNormal<3>::Vec arg;
    arg(0) = std::sqrt(std::pow(x - n.x_, 2) + std::pow(y - n.y_, 2));
    arg(1) = AngularDist(yaw, n.yaw_);
    arg(2) = 0.0;
    double denom = std::sqrt(std::pow(action.change_in_x_, 2) + std::pow(action.change_in_y_, 2));
    if (denom > 1e-6) {
      double action_dir_x = action.change_in_x_ / denom;
      double action_dir_y = action.change_in_y_ / denom;
      double angle_between_actions = std::acos(action_dir_x * n.action_dir_x_ + action_dir_y * n.action_dir_y_);
      arg(2) = angle_between_actions;
    }
    double weight =
      MultivariateNormal<3>::pdf(arg, mean, cov, false) / static_cast<double>(exec_error_neighborhoods_.size());
    penalty += weight;
  }

  penalty *= exec_error_params_.penalty_;

  return penalty;
}

bool Environment::UseWeightedPenalty() const { return use_weighted_penalty_; }

void Environment::SetUseWeightedPenalty(bool use) { use_weighted_penalty_ = use; }

Environment::DiscreteState::DiscreteState(int x_disc, int y_disc, int yaw_disc)
    : x_disc_(x_disc), y_disc_(y_disc), yaw_disc_(yaw_disc) {}

bool Environment::DiscreteState::operator==(const DiscreteState& other) const {
  return (x_disc_ == other.x_disc_ && y_disc_ == other.y_disc_ && yaw_disc_ == other.yaw_disc_);
}

std::size_t Environment::DiscreteState::HashFunction::operator()(const DiscreteState& state) const {
  std::size_t seed = 0;
  boost::hash_combine(seed, state.x_disc_);
  boost::hash_combine(seed, state.y_disc_);
  boost::hash_combine(seed, state.yaw_disc_);
  return seed;
}

std::string Environment::CollisionTestFunc(double x, double y, double yaw) {
  std::stringstream s;
  s << "at state (" << x << ", " << y << ", " << yaw << ")";
  robot_collision_object_->SetX(x);
  robot_collision_object_->SetY(y);
  robot_collision_object_->SetYaw(yaw);
  bool collision = false;
  for (auto col : collision_objects_) {
    if (col->InCollision(robot_collision_object_)) {
      s << " collision with " << col->GetName() << "!";

      s << " " << robot_collision_object_->GetName() << " vertices: [";
      auto robot_vs = robot_collision_object_->GetVertices();
      for (const auto& v : robot_vs) s << "(" << v.x() << ", " << v.y() << "), ";
      s << "], ";

      auto r = dynamic_cast<RectangleCollisionObject::Ptr>(col);
      if (r) {
        auto vs = r->GetVertices();
        s << " " << r->GetName() << " vertices: [";
        for (const auto& v : vs) s << "(" << v.x() << ", " << v.y() << "), ";
        s << "]";

        for (const auto& p : vs) {
          s << "\n===\n";
          for (int i = 0; i < 4; ++i) {
            const auto& first_vertex = robot_vs[i];
            const auto& second_vertex = robot_vs[(i + 1) % 4];
            Eigen::Vector2d first_vertex_to_point = p - first_vertex;
            Eigen::Vector2d edge = second_vertex - first_vertex;
            s << "\n    edge: (" << edge.x() << ", " << edge.y() << ")";
            s << "\n    vec2p: (" << first_vertex_to_point.x() << ", " << first_vertex_to_point.y() << ")";
            double cross_z = edge.x() * first_vertex_to_point.y() - first_vertex_to_point.x() * edge.y();
            s << "\n        cross: " << cross_z << "\n---\n";
          }
          s << "          ROBOT CONTAINS? " << (robot_collision_object_->Contains(p) ? "YES" : "NO") << "\n";
        }
        s << "\n==========\n==========\n";
        for (const auto& p : robot_vs) {
          s << "\n===\n";
          for (int i = 0; i < 4; ++i) {
            const auto& first_vertex = vs[i];
            const auto& second_vertex = vs[(i + 1) % 4];
            Eigen::Vector2d first_vertex_to_point = p - first_vertex;
            Eigen::Vector2d edge = second_vertex - first_vertex;
            s << "\n    edge: (" << edge.x() << ", " << edge.y() << ")";
            s << "\n    vec2p: (" << first_vertex_to_point.x() << ", " << first_vertex_to_point.y() << ")";
            double cross_z = edge.x() * first_vertex_to_point.y() - first_vertex_to_point.x() * edge.y();
            s << "\n        cross: " << cross_z << "\n---\n";
          }
          s << "          OBS CONTAINS? " << (r->Contains(p) ? "YES" : "NO") << "\n";
        }
      }

      collision = true;
      break;
    }
  }
  if (!collision) s << " no collisions";
  return s.str();
}

std::ostream& operator<<(std::ostream& os, const Environment::Action& action) {
  os << "{name: " << action.name_ << ", change_in_x: " << action.change_in_x_
     << ", change_in_y: " << action.change_in_y_ << ", change_in_yaw: " << action.change_in_yaw_
     << ", cost: " << action.cost_ << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Environment::ExecutionErrorNeighborhoodParameters& params) {
  os << "{state_radius_pos: " << params.state_radius_pos_ << ", state_radius_yaw: " << params.state_radius_yaw_
     << ", action_radius_pos: " << params.action_radius_pos_ << ", penalty: " << params.penalty_ << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Environment::ExecutionErrorNeighborhood& nbhd) {
  os << "{x: " << nbhd.x_ << ", y: " << nbhd.y_ << ", yaw: " << nbhd.yaw_ << ", action_dir_x: " << nbhd.action_dir_x_
     << ", action_dir_y: " << nbhd.action_dir_y_ << ", action_dir_yaw: " << nbhd.action_dir_yaw_ << "}";
  return os;
}

}  // namespace ellis_planner
