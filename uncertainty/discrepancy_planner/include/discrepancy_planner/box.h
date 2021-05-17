#ifndef DISCREPANCY_PLANNER_BOX_H
#define DISCREPANCY_PLANNER_BOX_H

#include <Eigen/Dense>
#include <discrepancy_planner/line_segment.h>
#include <discrepancy_planner/util.h>
#include <iostream>
#include <vector>

namespace discrepancy_planner {

// An oriented box in 3D.
class Box {
 public:
  Box(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
      const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity(), double size_x = 0, double size_y = 0,
      double size_z = 0);

  // Returns the position of the center of the box.
  const Eigen::Vector3d& GetPosition() const;

  // Sets the position of the center of the box.
  void SetPosition(const Eigen::Vector3d& position);

  // Returns the orientation of the box.
  const Eigen::Quaterniond& GetOrientation() const;

  // Sets the orientation of the box.
  void SetOrientation(const Eigen::Quaterniond& orientation);

  // Outputs the roll, pitch, and yaw of the box.
  void GetOrientationEuler(double& roll, double& pitch, double& yaw) const;

  // Sets the orientation of the box (using Euler angles).
  void SetOrientationEuler(double roll, double pitch, double yaw);

  // Get the size of the box along the (local) x-axis.
  double GetSizeX() const;

  // Get the size of the box along the (local) y-axis.
  double GetSizeY() const;

  // Get the size of the box along the (local) z-axis.
  double GetSizeZ() const;

  // Returns the corners of the box.
  std::array<Eigen::Vector3d, 8> GetCorners() const;

  std::array<LineSegment, 12> GetEdges() const;

  // Checks for the intersection with this box and another box.
  bool Intersects(const Box& other, Eigen::Vector3d* min_translation_vec = nullptr,
                  std::array<Eigen::Vector3d, 2>* basis = nullptr, std::vector<Eigen::Vector3d>* points = nullptr,
                  bool verbose = false) const;

  std::pair<double, double> Proj(const std::array<Eigen::Vector3d, 8>& corners, const Eigen::Vector3d& axis) const;

  double GetOverlap(const std::pair<double, double>& seg, const std::pair<double, double>& other_seg) const;

 private:
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;

  double size_x_;
  double size_y_;
  double size_z_;
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_BOX_H
