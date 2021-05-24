#ifndef DISCREPANCY_PLANNER_NORMAL_UTIL_H
#define DISCREPANCY_PLANNER_NORMAL_UTIL_H

#include <Eigen/Dense>

namespace discrepancy_planner {

template <unsigned int Dimension>
struct MultivariateNormal {
  typedef typename Eigen::Matrix<double, Dimension, 1> Vec;
  typedef typename Eigen::Matrix<double, Dimension, Dimension> Mat;

  static double PDF(const Vec& arg, const Vec& mean, const Mat& cov) {
    double n = std::pow(2 * M_PI, Dimension / 2) * std::sqrt(cov.determinant());
    return (
      std::exp(-0.5 * (arg - mean).transpose() * cov.inverse() * (arg - mean)) /
      n);
  }
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_NORMAL_UTIL_H
