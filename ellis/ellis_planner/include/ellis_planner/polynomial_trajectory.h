// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_POLYNOMIAL_TRAJECTORY_H_
#define ELLIS_PLANNER_POLYNOMIAL_TRAJECTORY_H_

#include <ellis_planner/polynomial.h>
#include <vector>

namespace ellis_planner {

template <unsigned int Dim>
class PolynomialTrajectory {
 public:
  struct Segment {
    Segment(double start_time, double end_time, const std::array<std::vector<double>, Dim>& coeffs);

    bool Contains(double time) const;

    bool Get(double time, int index, double& value, int deriv = 0) const;

    Segment Subseg(double start_time, double end_time) const;

    double start_time_;
    double end_time_;
    std::array<std::vector<double>, Dim> coeffs_;
  };

  PolynomialTrajectory(const std::vector<std::array<double, Dim>>& waypoints,
                       const std::vector<std::array<double, Dim>>& waypoint_vels,
                       const std::vector<double>& waypoint_times, bool return_boundary_waypoints = true);

  explicit PolynomialTrajectory(const std::vector<Segment>& segments, bool return_boundary_waypoints = true);

  // Gets the start time of the trajectory.
  bool GetStartTime(double& time) const;

  // Gets the end time of the trajectory.
  bool GetEndTime(double& time) const;

  // Gets the waypoint along the trajectory at time `time`.
  bool Get(double time, std::array<double, Dim>& waypoint, int deriv = 0, int* segment_index_ptr = nullptr) const;

  PolynomialTrajectory Subtraj(double start_time, double end_time) const;

  const std::vector<Segment>& GetSegments() const;

 private:
  std::vector<Segment> segments_;
  bool return_boundary_waypoints_;
};

}  // namespace ellis_planner

#include "ellis_planner/polynomial_trajectory_impl.h"

#endif  // ELLIS_PLANNER_POLYNOMIAL_TRAJECTORY_H_
