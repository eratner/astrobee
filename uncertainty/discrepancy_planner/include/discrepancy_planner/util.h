#ifndef DISCREPANCY_PLANNER_UTIL_H
#define DISCREPANCY_PLANNER_UTIL_H

#include <Eigen/Dense>
#include <angles/angles.h>
#include <ros/ros.h>

namespace discrepancy_planner {

template <typename Type, unsigned int Dim>
std::array<Type, Dim> DefaultValueArray(Type default_val) {
  std::array<Type, Dim> a;
  for (int i = 0; i < Dim; ++i) a[i] = default_val;

  return a;
}

// Converts an orientation specified via Euler angles `roll`, `pitch`, and `yaw` to an equivalent quaternion.
Eigen::Quaterniond RPYToQuaternion(double roll, double pitch, double yaw);

// Converts an orientation specified as the quaternion x`i + `y`j + `z`k + `w` to equivalent Euler angles `roll`,
// `pitch`, and `yaw`.
void QuaternionToRPY(double x, double y, double z, double w, double& roll, double& pitch, double& yaw);

// Finds the closest yaw `yaw_out`, assuming zero roll and pitch, to the orientation specified by the Euler angles
// `roll`, `pitch`, and `yaw`. Returns false if no such yaw exists, where the corresponding orientation is in within
// `threshold` radians of angular error of the orientation specified by `roll`, `pitch`, and `yaw`.
bool YawForZeroRollAndPitch(double roll, double pitch, double yaw, double& yaw_out, double threshold = 0.05);

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_UTIL_H
