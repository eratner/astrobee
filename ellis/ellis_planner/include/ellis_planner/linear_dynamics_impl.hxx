// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_LINEAR_DYNAMICS_IMPL_HXX_
#define ELLIS_PLANNER_LINEAR_DYNAMICS_IMPL_HXX_

#include <vector>

namespace ellis_planner {

template <unsigned int StateDim, unsigned int ControlDim>
LinearDynamics<StateDim, ControlDim>::LinearDynamics(const Eigen::Matrix<double, StateDim, StateDim>& A,
                                                     const Eigen::Matrix<double, StateDim, ControlDim>& B,
                                                     const Eigen::Matrix<double, StateDim, StateDim>& Bd)
    : A_(A), B_(B), Bd_(Bd) {}

template <unsigned int StateDim, unsigned int ControlDim>
LinearDynamics<StateDim, ControlDim>::~LinearDynamics() {}

template <unsigned int StateDim, unsigned int ControlDim>
typename LinearDynamics<StateDim, ControlDim>::StateVec LinearDynamics<StateDim, ControlDim>::Step(
  const StateVec& x, const ControlVec& u) const {
  // TODO(eratner) Optionally add disturbance
  return (A_ * x + B_ * u);
}

template <unsigned int StateDim, unsigned int ControlDim>
void LinearDynamics<StateDim, ControlDim>::Predict(const StateVec& start_state, const std::vector<ControlVec>& controls,
                                                   std::vector<StateVec>& pred_mean,
                                                   std::vector<Eigen::Matrix<double, StateDim, StateDim>>& pred_cov) {
  Eigen::Matrix<double, StateDim + ControlDim, 1> z_start;
  z_start.template block<StateDim, 1>(0, 0) = start_state;
  z_start.template block<ControlDim, 1>(StateDim, 0) = controls[0];

  std::vector<Eigen::Matrix<double, StateDim + ControlDim, 1>> z_pred_mean = {z_start};
  std::vector<Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>> z_pred_cov = {
    Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>::Zero()};

  Eigen::Matrix<double, 2 * StateDim, StateDim + ControlDim> A_z =
    Eigen::Matrix<double, 2 * StateDim, StateDim + ControlDim>::Identity();
  A_z.template block<StateDim, StateDim>(0, 0) = A_;
  A_z.template block<StateDim, ControlDim>(0, StateDim) = B_;

  for (unsigned int t = 1; t <= controls.size(); ++t) {
    Eigen::Matrix<double, StateDim + ControlDim, 1> mean = z_pred_mean.back();
    mean.template block<StateDim, 1>(StateDim, 0) = controls[t - 1];
    const auto& cov = z_pred_cov.back();

    // Predict the mean.
    Eigen::Matrix<double, StateDim + ControlDim, 1> d_mean = Eigen::Matrix<double, StateDim + ControlDim, 1>::Zero();
    for (unsigned int i = 0; i < StateDim; ++i) {
      d_mean(i) = disturbance_[i].MeanFunc(mean);
    }

    d_mean.template block<StateDim, 1>(0, 0) = Bd_ * d_mean.template block<StateDim, 1>(0, 0);

    // std::cout << "A_z: \n" << A_z << std::endl;
    // std::cout << "mean: \n" << mean << std::endl;
    Eigen::Matrix<double, StateDim + ControlDim, 1> next_mean = A_z * mean + d_mean;
    z_pred_mean.push_back(next_mean);

    // Predict the covariance.
    Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim> next_cov =
      Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>::Zero();
    if (t == 1) {
      // Base case.
      for (unsigned int i = 0; i < StateDim; ++i) {
        next_cov(i, i) = disturbance_[i].VarFunc(mean);
      }

      next_cov.template block<StateDim, StateDim>(0, 0) =
        Bd_ * next_cov.template block<StateDim, StateDim>(0, 0) * Bd_.transpose();
    } else {
      // Recursive case.
      next_cov = A_z * cov * A_z.transpose();

      Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim> V =
        Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>::Zero();
      for (unsigned int i = 0; i < StateDim; ++i) {
        const auto& disturbance = disturbance_[i];

        V(i, i) = disturbance.VarFunc(mean);
        V(i, i) += (0.5 * (disturbance.GetSecondDerivOfVarFunc(mean) * cov).trace());
        Eigen::Matrix<double, StateDim + ControlDim, 1> deriv_mean = disturbance.GetFirstDerivOfMeanFunc(mean);
        V(i, i) += (deriv_mean.transpose() * cov * deriv_mean);
      }

      V.template block<StateDim, StateDim>(0, 0) = Bd_ * V.template block<StateDim, StateDim>(0, 0) * Bd_.transpose();

      next_cov += V;
    }
    z_pred_cov.push_back(next_cov);
  }

  for (int i = 0; i < z_pred_mean.size(); ++i) {
    pred_mean.push_back(z_pred_mean[i].template block<StateDim, 1>(0, 0));
    pred_cov.push_back(z_pred_cov[i].template block<StateDim, StateDim>(0, 0));
  }
}

template <unsigned int StateDim, unsigned int ControlDim>
void LinearDynamics<StateDim, ControlDim>::Predict(
  const StateVec& start_state, const std::vector<ControlVec>& controls, std::vector<StateVec>& pred_mean,
  std::vector<Eigen::Matrix<double, StateDim, StateDim>>& pred_cov,
  const std::array<GP<StateDim + ControlDim>*, StateDim>& disturbances) const {
  Eigen::Matrix<double, StateDim + ControlDim, 1> z_start;
  z_start.template block<StateDim, 1>(0, 0) = start_state;
  z_start.template block<ControlDim, 1>(StateDim, 0) = controls[0];

  std::vector<Eigen::Matrix<double, StateDim + ControlDim, 1>> z_pred_mean = {z_start};
  std::vector<Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>> z_pred_cov = {
    Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>::Zero()};

  Eigen::Matrix<double, 2 * StateDim, StateDim + ControlDim> A_z =
    Eigen::Matrix<double, 2 * StateDim, StateDim + ControlDim>::Identity();
  A_z.template block<StateDim, StateDim>(0, 0) = A_;
  A_z.template block<StateDim, ControlDim>(0, StateDim) = B_;

  for (unsigned int t = 1; t <= controls.size(); ++t) {
    Eigen::Matrix<double, StateDim + ControlDim, 1> mean = z_pred_mean.back();
    mean.template block<StateDim, 1>(StateDim, 0) = controls[t - 1];
    const auto& cov = z_pred_cov.back();

    // Predict the mean.
    Eigen::Matrix<double, StateDim + ControlDim, 1> d_mean = Eigen::Matrix<double, StateDim + ControlDim, 1>::Zero();
    for (unsigned int i = 0; i < StateDim; ++i) {
      d_mean(i) = disturbances[i]->MeanFunc(mean);
    }

    d_mean.template block<StateDim, 1>(0, 0) = Bd_ * d_mean.template block<StateDim, 1>(0, 0);

    // std::cout << "A_z: \n" << A_z << std::endl;
    // std::cout << "mean: \n" << mean << std::endl;
    Eigen::Matrix<double, StateDim + ControlDim, 1> next_mean = A_z * mean + d_mean;
    z_pred_mean.push_back(next_mean);

    // Predict the covariance.
    Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim> next_cov =
      Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>::Zero();
    if (t == 1) {
      // Base case.
      for (unsigned int i = 0; i < StateDim; ++i) {
        next_cov(i, i) = disturbances[i]->VarFunc(mean);
      }

      next_cov.template block<StateDim, StateDim>(0, 0) =
        Bd_ * next_cov.template block<StateDim, StateDim>(0, 0) * Bd_.transpose();
    } else {
      // Recursive case.
      next_cov = A_z * cov * A_z.transpose();

      Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim> V =
        Eigen::Matrix<double, StateDim + ControlDim, StateDim + ControlDim>::Zero();
      for (unsigned int i = 0; i < StateDim; ++i) {
        const auto* disturbance = disturbances[i];

        V(i, i) = disturbance->VarFunc(mean);
        V(i, i) += (0.5 * (disturbance->GetSecondDerivOfVarFunc(mean) * cov).trace());
        Eigen::Matrix<double, StateDim + ControlDim, 1> deriv_mean = disturbance->GetFirstDerivOfMeanFunc(mean);
        V(i, i) += (deriv_mean.transpose() * cov * deriv_mean);
      }

      V.template block<StateDim, StateDim>(0, 0) = Bd_ * V.template block<StateDim, StateDim>(0, 0) * Bd_.transpose();

      next_cov += V;
    }
    z_pred_cov.push_back(next_cov);
  }

  for (int i = 0; i < z_pred_mean.size(); ++i) {
    pred_mean.push_back(z_pred_mean[i].template block<StateDim, 1>(0, 0));
    pred_cov.push_back(z_pred_cov[i].template block<StateDim, StateDim>(0, 0));
  }
}

template <unsigned int StateDim, unsigned int ControlDim>
std::array<GP<StateDim + ControlDim>, StateDim>& LinearDynamics<StateDim, ControlDim>::GetDisturbances() {
  return disturbance_;
}

template <unsigned int StateDim, unsigned int ControlDim>
const Eigen::Matrix<double, StateDim, StateDim>& LinearDynamics<StateDim, ControlDim>::GetA() const {
  return A_;
}

template <unsigned int StateDim, unsigned int ControlDim>
const Eigen::Matrix<double, StateDim, ControlDim>& LinearDynamics<StateDim, ControlDim>::GetB() const {
  return B_;
}

template <unsigned int StateDim, unsigned int ControlDim>
const Eigen::Matrix<double, StateDim, StateDim>& LinearDynamics<StateDim, ControlDim>::GetBd() const {
  return Bd_;
}

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_LINEAR_DYNAMICS_IMPL_HXX_
