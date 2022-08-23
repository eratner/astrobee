// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_LINEAR_DYNAMICS_H_
#define ELLIS_PLANNER_LINEAR_DYNAMICS_H_

#include <ellis_planner/gp.h>
#include <array>
#include <vector>

namespace ellis_planner {

template <unsigned int StateDim, unsigned int ControlDim>
class LinearDynamics {
 public:
  typedef Eigen::Matrix<double, StateDim, 1> StateVec;
  typedef Eigen::Matrix<double, ControlDim, 1> ControlVec;

  explicit LinearDynamics(
    const Eigen::Matrix<double, StateDim, StateDim>& A = Eigen::Matrix<double, StateDim, StateDim>::Identity(),
    const Eigen::Matrix<double, StateDim, ControlDim>& B = Eigen::Matrix<double, StateDim, ControlDim>::Identity());

  ~LinearDynamics();

  StateVec Step(const StateVec& x, const ControlVec& u) const;

  void Predict(const StateVec& start_state, const std::vector<ControlVec>& controls, std::vector<StateVec>& pred_mean,
               std::vector<Eigen::Matrix<double, StateDim, StateDim>>& pred_cov);

  std::array<GP<StateDim + ControlDim>, StateDim> &GetDisturbances();

  const Eigen::Matrix<double, StateDim, StateDim> &GetA() const;

  const Eigen::Matrix<double, StateDim, ControlDim> &GetB() const;

 private:
  Eigen::Matrix<double, StateDim, StateDim> A_;
  Eigen::Matrix<double, StateDim, ControlDim> B_;
  std::array<GP<StateDim + ControlDim>, StateDim> disturbance_;
};

}  // namespace ellis_planner

#include "ellis_planner/linear_dynamics_impl.hxx"

#endif  // ELLIS_PLANNER_LINEAR_DYNAMICS_H_
