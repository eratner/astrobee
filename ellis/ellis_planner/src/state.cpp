// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/state.h>

namespace ellis_planner {

State::State(unsigned int id, double x, double y, double yaw)
    : id_(id), x_(x), y_(y), yaw_(yaw), cost_to_come_(1e9), parent_(nullptr) {}

unsigned int State::GetId() const { return id_; }

double State::GetX() const { return x_; }

double State::GetY() const { return y_; }

double State::GetYaw() const { return yaw_; }

std::ostream& operator<<(std::ostream& os, const State& state) {
  os << "{id: " << state.GetId() << ", x: " << state.GetX() << ", y: " << state.GetY() << ", yaw: " << state.GetYaw()
     << ", cost_to_come: " << state.cost_to_come_ << ", parent_id: " << (state.parent_ ? state.parent_->GetId() : -1)
     << "}";
  return os;
}

}  // namespace ellis_planner
