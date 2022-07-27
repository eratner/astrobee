// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_NORMAL_DIST_H_
#define ELLIS_PLANNER_NORMAL_DIST_H_

#include <Eigen/Dense>

namespace ellis_planner {

template <unsigned int Dimension>
struct MultivariateNormal {
  typedef typename Eigen::Matrix<double, Dimension, 1> Vec;
  typedef typename Eigen::Matrix<double, Dimension, Dimension> Mat;

  static double pdf(const Vec& arg, const Vec& mean, const Mat& cov, bool normalized = true) {
    double n = 1.0;
    if (normalized) n = std::pow(2 * M_PI, Dimension / 2) * std::sqrt(cov.determinant());

    return (std::exp(-0.5 * (arg - mean).transpose() * cov.inverse() * (arg - mean)) / n);
  }
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_NORMAL_DIST_H_
