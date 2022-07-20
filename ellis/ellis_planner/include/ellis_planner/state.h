// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_STATE_H_
#define ELLIS_PLANNER_STATE_H_

#include <iostream>

namespace ellis_planner {

constexpr unsigned int kStateDim = 3;  // (x, y, yaw)

class State {
 public:
  typedef State* Ptr;

  State(unsigned int id, double x, double y, double yaw);

  unsigned int GetId() const;

  double GetX() const;

  double GetY() const;

  double GetYaw() const;

  // TODO(eratner) This shouldn't be public
  // Search info.
  double cost_to_come_;
  Ptr parent_;

 private:
  unsigned int id_;
  double x_;
  double y_;
  double yaw_;
};

std::ostream& operator<<(std::ostream& os, const State& state);

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_STATE_H_
