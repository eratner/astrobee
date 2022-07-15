// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_POLYNOMIAL_TRAJECTORY_IMPL_H_
#define ELLIS_PLANNER_POLYNOMIAL_TRAJECTORY_IMPL_H_

#include <vector>

namespace ellis_planner {

template <unsigned int Dim>
PolynomialTrajectory<Dim>::Segment::Segment(double start_time, double end_time,
                                            const std::array<std::vector<double>, Dim>& coeffs)
    : start_time_(start_time), end_time_(end_time), coeffs_(coeffs) {}

template <unsigned int Dim>
bool PolynomialTrajectory<Dim>::Segment::Contains(double time) const {
  return (time >= start_time_ && time <= end_time_);
}

template <unsigned int Dim>
bool PolynomialTrajectory<Dim>::Segment::Get(double time, int index, double& value, int deriv) const {
  if (!Contains(time)) return false;

  if (index < 0 || index >= Dim) return false;

  const auto& coeff = coeffs_[index];
  value = Polynomial::Value(time - start_time_, 3, coeff, deriv);
  return true;
}

template <unsigned int Dim>
typename PolynomialTrajectory<Dim>::Segment PolynomialTrajectory<Dim>::Segment::Subseg(double start_time,
                                                                                       double end_time) const {
  // TODO(eratner) Error check -- make sure start_time and end_time valid.
  std::vector<double> start_and_end_times(2);
  start_and_end_times[0] = 0;
  start_and_end_times[1] = end_time - start_time;

  std::array<std::vector<double>, Dim> coeffs;
  for (int j = 0; j < Dim; ++j) {
    std::vector<double> boundary_values(4);
    boundary_values[0] = Polynomial::Value(start_time - start_time_, 3, coeffs_[j], 0);  // Start position.
    boundary_values[1] = Polynomial::Value(start_time - start_time_, 3, coeffs_[j], 1);  // Start velocity.
    boundary_values[2] = Polynomial::Value(end_time - start_time_, 3, coeffs_[j], 0);    // End position.
    boundary_values[3] = Polynomial::Value(end_time - start_time_, 3, coeffs_[j], 1);    // End velocity.
    coeffs[j] = Polynomial::GetCoefficients(3, start_and_end_times, boundary_values);
  }

  return Segment(start_time, end_time, coeffs);
}

template <unsigned int Dim>
PolynomialTrajectory<Dim>::PolynomialTrajectory(const std::vector<std::array<double, Dim>>& waypoints,
                                                const std::vector<std::array<double, Dim>>& waypoint_vels,
                                                const std::vector<double>& waypoint_times,
                                                bool return_boundary_waypoints)
    : return_boundary_waypoints_(return_boundary_waypoints) {
  // TODO(eratner) Error check the input vector sizes
  if (waypoints.size() > 1) {
    for (int i = 0; i < waypoints.size() - 1; ++i) {
      const auto& start_waypoint = waypoints[i];
      const auto& start_waypoint_vel = waypoint_vels[i];
      const auto& end_waypoint = waypoints[i + 1];
      const auto& end_waypoint_vel = waypoint_vels[i + 1];
      auto start_time = waypoint_times[i];
      auto end_time = waypoint_times[i + 1];

      // Fit the polynomials.
      std::vector<double> start_and_end_times(2);
      start_and_end_times[0] = 0;
      start_and_end_times[1] = end_time - start_time;

      std::array<double, Dim> end_velocity;
      std::array<std::vector<double>, Dim> coeffs;
      for (int j = 0; j < Dim; ++j) {
        std::vector<double> boundary_values(4);
        boundary_values[0] = start_waypoint[j];      // Start position.
        boundary_values[1] = start_waypoint_vel[j];  // Start velocity.
        boundary_values[2] = end_waypoint[j];        // End position.
        boundary_values[3] = end_waypoint_vel[j];    // End velocity.

        // Get the coefficients.
        coeffs[j] = Polynomial::GetCoefficients(3, start_and_end_times, boundary_values);
      }

      segments_.emplace_back(start_time, end_time, coeffs);
    }
  }
}

template <unsigned int Dim>
PolynomialTrajectory<Dim>::PolynomialTrajectory(const std::vector<Segment>& segments, bool return_boundary_waypoints)
    : segments_(segments), return_boundary_waypoints_(return_boundary_waypoints) {}

template <unsigned int Dim>
bool PolynomialTrajectory<Dim>::GetStartTime(double& time) const {
  if (segments_.empty()) return false;

  time = segments_[0].start_time_;
  return true;
}

template <unsigned int Dim>
bool PolynomialTrajectory<Dim>::GetEndTime(double& time) const {
  if (segments_.empty()) return false;

  time = segments_.back().end_time_;
  return true;
}

template <unsigned int Dim>
bool PolynomialTrajectory<Dim>::Get(double time, std::array<double, Dim>& waypoint, int deriv,
                                    int* segment_index_ptr) const {
  double start_time, end_time;
  if (!GetStartTime(start_time) || !GetEndTime(end_time)) return false;

  if (time < start_time || time > end_time) {
    if (return_boundary_waypoints_) {
      const auto& segment = segments_[(time < start_time ? 0 : segments_.size() - 1)];
      for (int i = 0; i < Dim; ++i) {
        double value;
        if (!segment.Get((time < start_time ? segment.start_time_ : segment.end_time_), i, value, deriv)) return false;

        waypoint[i] = value;
      }

      return true;
    }

    return false;
  }

  // Find the correct segment.
  int segment_index = -1;
  for (int i = 0; i < segments_.size(); ++i) {
    const auto& segment = segments_[i];
    if (segment.Contains(time)) {
      segment_index = i;
      break;
    }
  }

  if (segment_index < 0) return false;

  if (segment_index_ptr) *segment_index_ptr = segment_index;

  const auto& segment = segments_[segment_index];
  for (int i = 0; i < Dim; ++i) {
    double value;
    if (!segment.Get(time, i, value, deriv)) return false;

    waypoint[i] = value;
  }

  return true;
}

template <unsigned int Dim>
PolynomialTrajectory<Dim> PolynomialTrajectory<Dim>::Subtraj(double start_time, double end_time) const {
  int start_index = -1, end_index = -1;
  for (int i = 0; i < segments_.size(); ++i) {
    const auto& s = segments_[i];
    if (s.Contains(start_time)) start_index = i;
    if (s.Contains(end_time)) end_index = i;

    if (start_index >= 0 && end_index >= 0) break;
  }

  std::vector<Segment> segments;
  if (start_index < 0) {
    ROS_WARN_STREAM(
      "[PolynomialTrajectory::Subtraj] Could not find "
      "segment for start time "
      << start_time << " sec");
  } else if (end_index < 0) {
    ROS_WARN_STREAM("[PolynomialTrajectory::Subtraj] Could not find segment for end time " << end_time << " sec");
  } else {
    // TODO(eratner) More error checking -- what if start_index > end_index?
    for (int i = start_index; i <= end_index; ++i) {
      if (i == start_index) {
        segments.push_back(segments_[i].Subseg(start_time, segments_[i].end_time_));
      } else if (i == end_index) {
        segments.push_back(segments_[i].Subseg(segments_[i].start_time_, end_time));
      } else {
        segments.push_back(segments_[i]);
      }
    }
  }

  return PolynomialTrajectory(segments, return_boundary_waypoints_);
}

template <unsigned int Dim>
const std::vector<typename PolynomialTrajectory<Dim>::Segment>& PolynomialTrajectory<Dim>::GetSegments() const {
  return segments_;
}

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_POLYNOMIAL_TRAJECTORY_IMPL_H_
