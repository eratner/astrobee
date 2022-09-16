// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_GP_H_
#define ELLIS_PLANNER_GP_H_

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <array>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <iostream>

namespace ellis_planner {

template <unsigned int InputDim>
class GP {
 public:
  struct Parameters {
    Parameters();

    std::string ToYaml() const;

    double v0_;                             // Variance of noise.
    double v1_;                             // Parameter of squared exponential covariance function.
    std::array<double, InputDim> weights_;  // Parameter of squared exponential covariance function.
  };

  typedef Eigen::Matrix<double, InputDim, 1> InputVec;

  explicit GP(const Parameters& params = Parameters());

  ~GP();

  void Reset();

  void Train(const std::vector<InputVec>& inputs, const std::vector<double>& targets);

  double CovFunc(const InputVec& xp, const InputVec& xq) const;

  double MeanFunc(const InputVec& x) const;

  double VarFunc(const InputVec& x) const;

  Eigen::VectorXd GetCovWithTrainingInputs(const InputVec& x) const;

  Eigen::Matrix<double, InputDim, 1> GetFirstDerivOfMeanFunc(const InputVec& x) const;

  Eigen::Matrix<double, InputDim, InputDim> GetSecondDerivOfVarFunc(const InputVec& x) const;

  Parameters& GetParameters();

  unsigned int GetNumTrainingInputs() const;

  const Eigen::MatrixXd &GetK() const;

 private:
  Parameters params_;

  std::vector<InputVec> training_inputs_;
  std::vector<double> training_targets_;
  Eigen::VectorXd y_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd K_inv_;
  Eigen::LLT<Eigen::MatrixXd> K_decomp_;
  Eigen::VectorXd K_inv_y_;
};

}  // namespace ellis_planner

#include "ellis_planner/gp_impl.hxx"

#endif  // ELLIS_PLANNER_GP_H_