// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_GP_IMPL_HXX_
#define ELLIS_PLANNER_GP_IMPL_HXX_

#include <vector>

namespace ellis_planner {

template <unsigned int InputDim>
GP<InputDim>::Parameters::Parameters() : v0_(1e-2), v1_(1.0) {
  for (unsigned int i = 0; i < InputDim; ++i) weights_[i] = 1.0;
}

template <unsigned int InputDim>
GP<InputDim>::GP(const Parameters& params) : params_(params) {}

template <unsigned int InputDim>
GP<InputDim>::~GP() {}

template <unsigned int InputDim>
void GP<InputDim>::Reset() {
  training_inputs_.clear();
}

template <unsigned int InputDim>
void GP<InputDim>::Train(const std::vector<InputVec>& inputs, const std::vector<double>& targets) {
  // TODO(eratner) Refactor below to allow for ADDING training data, rather than overwriting it
  // Store the training inputs and targets.
  training_inputs_ = inputs;
  y_ = Eigen::VectorXd::Zero(targets.size());
  for (unsigned int i = 0; i < targets.size(); ++i) y_(i) = targets[i];

  // Compute the covariance matrix of the inputs.
  K_ = Eigen::MatrixXd::Zero(inputs.size(), inputs.size());
  for (unsigned int i = 0; i < inputs.size(); ++i) {
    for (unsigned int j = 0; j < inputs.size(); ++j) {
      K_(i, j) = CovFunc(inputs[i], inputs[j]);
    }
  }

  K_ += (params_.v0_ * Eigen::MatrixXd::Identity(inputs.size(), inputs.size()));

  // Compute the Cholesky decomposition of the matrix (for later use).
  K_decomp_ = K_.llt();
  K_inv_y_ = K_decomp_.solve(y_);

  std::cout << "K: \n" << K_ << std::endl;
  std::cout << "K_inv_y: \n" << K_inv_y_ << std::endl;
}

template <unsigned int InputDim>
double GP<InputDim>::CovFunc(const InputVec& xp, const InputVec& xq) const {
  double s = 0.0;
  for (unsigned int i = 0; i < InputDim; ++i) {
    s += (std::pow(xp(i) - xq(i), 2) / std::pow(params_.weights_[i], 2));
  }

  return (params_.v1_ * std::exp(-0.5 * s));
}

template <unsigned int InputDim>
double GP<InputDim>::MeanFunc(const InputVec& x) const {
  Eigen::VectorXd k = GetCovWithTrainingInputs(x);
  return (k.transpose() * K_inv_y_);
}

template <unsigned int InputDim>
double GP<InputDim>::VarFunc(const InputVec& x) const {
  Eigen::VectorXd k = GetCovWithTrainingInputs(x);
  Eigen::VectorXd K_inv_k = K_decomp_.solve(k);
  return (CovFunc(x, x) - k.transpose() * K_inv_k);
}

template <unsigned int InputDim>
Eigen::VectorXd GP<InputDim>::GetCovWithTrainingInputs(const InputVec& x) const {
  Eigen::VectorXd k = Eigen::VectorXd::Zero(training_inputs_.size());
  for (unsigned int i = 0; i < training_inputs_.size(); ++i) {
    k(i) = CovFunc(training_inputs_[i], x);
  }
  return k;
}

template <unsigned int InputDim>
Eigen::Matrix<double, InputDim, 1> GP<InputDim>::GetFirstDerivOfMeanFunc(const InputVec& x) const {
  Eigen::VectorXd k = GetCovWithTrainingInputs(x);

  Eigen::Matrix<double, InputDim, 1> deriv = Eigen::Matrix<double, InputDim, 1>::Zero();
  for (unsigned int d = 0; d < InputDim; ++d) {
    Eigen::VectorXd xd(training_inputs_.size());
    for (unsigned int i = 0; i < training_inputs_.size(); ++i) {
      xd(i) = training_inputs_[i](d) - x(d);
    }
    deriv[d] = params_.weights_[d] * xd.cwiseProduct(k).transpose() * K_inv_y_;
  }
  return deriv;
}

template <unsigned int InputDim>
Eigen::Matrix<double, InputDim, InputDim> GP<InputDim>::GetSecondDerivOfVarFunc(const InputVec& x) const {
  // TODO(eratner) Refactor to avoid inverting K directly
  Eigen::MatrixXd K_inv = K_.inverse();

  Eigen::VectorXd k = GetCovWithTrainingInputs(x);

  Eigen::Matrix<double, InputDim, InputDim> deriv = Eigen::Matrix<double, InputDim, InputDim>::Zero();
  for (unsigned int d = 0; d < InputDim; ++d) {
    for (unsigned int e = 0; e < InputDim; ++e) {
      Eigen::VectorXd xd(training_inputs_.size());
      Eigen::VectorXd xe(training_inputs_.size());
      for (unsigned int i = 0; i < training_inputs_.size(); ++i) {
        xd(i) = training_inputs_[i](d) - x(d);
        xe(i) = training_inputs_[i](e) - x(e);
      }

      deriv(d, e) = xd.cwiseProduct(k).transpose() * K_inv * xe.cwiseProduct(k);
      deriv(d, e) += xd.cwiseProduct(xe.cwiseProduct(k)).transpose() * K_inv * k;
      deriv(d, e) *= -2.0 * params_.weights_[d] * params_.weights_[e];
      if (d == e) deriv(d, e) += (2.0 * params_.weights_[d] * k.transpose() * K_inv * k);
    }
  }

  return deriv;
}

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_GP_IMPL_HXX_
