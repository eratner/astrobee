#ifndef DISCREPANCY_PLANNER_LINE_SEGMENT_H
#define DISCREPANCY_PLANNER_LINE_SEGMENT_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>

namespace discrepancy_planner {

class LineSegment {
 public:
  LineSegment(const Eigen::Vector3d& first_point = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& second_point = Eigen::Vector3d::Zero());

  const Eigen::Vector3d& GetFirstPoint() const;

  const Eigen::Vector3d& GetSecondPoint() const;

  // Returns true if there is an intersection between this line segment and the
  // specified line segment, and outputs the intersection point. Returns false
  // otherwise.
  bool GetIntersection(const LineSegment& segment, Eigen::Vector3d& intersection) const;

  // Returns the distance between the two closest points on the this line
  // segment and the other line segment. Also outputs the actual closest points
  // on this line segment and on the other line segment.
  double GetClosestPoints(const LineSegment& segment, Eigen::Vector3d& point, Eigen::Vector3d& other_point) const;

  // Returns the closest point on this line segment to `point`.
  Eigen::Vector3d GetClosestPoint(const Eigen::Vector3d& point) const;

 private:
  Eigen::Vector3d first_point_;
  Eigen::Vector3d second_point_;
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_LINE_SEGMENT_H
