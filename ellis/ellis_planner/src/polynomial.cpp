// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/polynomial.h>
#include <vector>

namespace ellis_planner {

std::vector<double> Polynomial::GetCoefficients(int order, const std::vector<double>& start_and_end_times,
                                                const std::vector<double>& boundary_values) {
  if (start_and_end_times.size() != 2) {
    throw std::invalid_argument("Must specify exactly 1 start time and 1 end time.");
  }

  if (boundary_values.size() != (order + 1)) {
    throw std::invalid_argument("For polynomial of order " + std::to_string(order) + ", must specify " +
                                std::to_string(order + 1) + " boundary values.");
  }

  Eigen::MatrixXd A(order + 1, order + 1);
  Eigen::MatrixXd b(order + 1, 1);

  for (int t = 0; t < 2; ++t) {
    int i_offset = t * (order + 1) / 2;

    for (int i = 0; i < (order + 1) / 2; ++i) {
      for (int j = 0; j < order + 1; ++j) {
        int exponent = order - i - j;
        if (exponent < 0) {
          A(i + i_offset, j) = 0.0;
        } else {
          double mult = 1.0;

          for (int k = 0; k < i; ++k) mult *= (order - j - k);

          A(i + i_offset, j) = mult * std::pow(start_and_end_times[t], exponent);
        }
      }
    }
  }

  for (int i = 0; i < order + 1; ++i) {
    b(i, 0) = boundary_values[i];
  }

  std::vector<double> coefficients;
  Eigen::VectorXd coeffs = A.inverse() * b;
  for (int i = 0; i < order + 1; ++i) coefficients.push_back(coeffs(i, 0));

  return coefficients;
}

double Polynomial::Value(double t, int order, const std::vector<double>& coefficients, int deriv) {
  double value = 0;

  for (int i = 0; i < order + 1; ++i) {
    int exponent = order - i - deriv;
    if (exponent >= 0) {
      double mult = coefficients[i];

      for (int k = 0; k < deriv; ++k) mult *= (order - i - k);

      value += mult * std::pow(t, exponent);
    }
  }

  return value;
}

}  // namespace ellis_planner
