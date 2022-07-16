// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/environment.h>
#include <string>
#include <vector>

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

Environment::Environment()
    : m_per_unit_x_(1e-2),
      m_per_unit_y_(1e-2),
      rad_per_unit_yaw_(8e-2),
      goal_x_(0.0),
      goal_y_(0.0),
      goal_yaw_(0.0),
      goal_pos_tol_(1e-1),
      goal_ang_tol_(1e-1) {}

Environment::~Environment() { Clear(); }

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

void Environment::SetActions(const std::vector<Action>& actions) { actions_ = actions; }

const std::vector<Environment::Action>& Environment::GetActions() const { return actions_; }

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

  // TODO(eratner) Collision checking goes here

  auto outcome = GetState(x, y, yaw);
  return std::make_tuple(outcome, action.cost_);
}

double Environment::GetHeuristicCostToGoal(const State::Ptr state) const {
  if (IsGoal(state)) return 0;

  // TODO(eratner) Because of goal tolerances, this could slightly overestimate the cost
  return (std::abs(goal_x_ - state->GetX()) + std::abs(goal_y_ - state->GetY()));
}

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

}  // namespace ellis_planner
