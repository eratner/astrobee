// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_NORMAL_DIST_H_
#define ELLIS_PLANNER_NORMAL_DIST_H_

#include <Eigen/Dense>
#include <random>
#include <vector>

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

  // See https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
  static std::vector<Vec> Sample(const Vec& mean, const Mat& cov, unsigned int num_samples = 1) {
    std::vector<Vec> samples;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d;

    Eigen::SelfAdjointEigenSolver<Mat> solver(cov);
    Mat transform = solver.eigenvectors() * solver.eigenvalues().cwiseSqrt().asDiagonal();

    for (unsigned int i = 0; i < num_samples; ++i) {
      // TODO(eratner) Unapproved C++ feature??
      // Vec sample = mean + transform * Vec{}.unaryExpr([&](double x) { return d(gen); });
      Vec m;
      for (unsigned int j = 0; j < Dimension; ++j) m(j) = d(gen);
      Vec sample = mean + transform * m;
      samples.push_back(sample);
    }

    return samples;
  }
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_NORMAL_DIST_H_
