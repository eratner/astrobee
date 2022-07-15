// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_POLYNOMIAL_H_
#define ELLIS_PLANNER_POLYNOMIAL_H_

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>

namespace ellis_planner {

// Functionality for working with polynomials.
class Polynomial {
 public:
  // Computes the coefficients for a polynomial of order `order`, given the
  // boundary conditions specified in `boundary_values` at the start and end
  // times given by `start_and_end_times[0]` and `start_and_end_times[1]`,
  // respectively.
  //
  // Example:
  //   start_and_end_times[0] = 0; // Start time.
  //   start_and_end_times[1] = 1; // End time.
  //   boundary_values[0] = 0;     // Polynomial's value at start time.
  //   boundary_values[1] = 0;     // Polynomial's derivative at start time.
  //   boundary_values[2] = 2;     // Polynomial's value at end time.
  //   boundary_values[3] = 0;     // Polynomial's derivative at end time.
  //   // Fit a 3rd-order polynomial to the boundary conditions.
  //   std::vector<double> coeffs = Polynomial::GetCoefficients(
  //     3, start_and_end_times, boundary_values);
  static std::vector<double> GetCoefficients(int order, const std::vector<double>& start_and_end_times,
                                             const std::vector<double>& boundary_values);

  // Computes the value of the (`deriv` order derivative of) a polynomial of
  // order `order` specified by `coefficients` at time `t`.
  static double Value(double t, int order, const std::vector<double>& coefficients, int deriv = 0);
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_POLYNOMIAL_H_
