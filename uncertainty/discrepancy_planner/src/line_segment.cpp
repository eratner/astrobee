#include <discrepancy_planner/line_segment.h>

namespace discrepancy_planner {

LineSegment::LineSegment(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point)
    : first_point_(first_point), second_point_(second_point) {}

const Eigen::Vector3d& LineSegment::GetFirstPoint() const { return first_point_; }

const Eigen::Vector3d& LineSegment::GetSecondPoint() const { return second_point_; }

bool LineSegment::GetIntersection(const LineSegment& segment, Eigen::Vector3d& intersection) const {
  intersection = Eigen::Vector3d::Zero();

  double t = -1.0;

  auto delta = first_point_ - segment.GetFirstPoint();
  auto r1 = second_point_ - first_point_;
  auto r2 = segment.GetSecondPoint() - segment.GetFirstPoint();

  double denom = r1(0) * r2(1) - r1(1) * r2(0);
  if (std::abs(denom) > 1e-6) {
    t = (delta(1) * r2(0) - delta(0) * r2(1)) / denom;
  } else {
    // Try another denominator.
    denom = r1(1) * r2(2) - r1(2) * r2(1);
    if (std::abs(denom) > 1e-6) {
      t = (delta(2) * r2(1) - delta(1) * r2(2)) / denom;
    } else {
      // Try another denominator.
      denom = r1(2) * r2(0) - r1(0) * r2(2);
      if (std::abs(denom) > 1e-6) {
        t = (delta(0) * r2(2) - delta(2) * r2(0)) / denom;
      } else {
        std::cout << "No intersection!" << std::endl;
        return false;
      }
    }
  }

  if (t < 0 || t > 1) {
    std::cout << "Intersection point is past the extents of the line segment" << std::endl;
    return false;
  }

  intersection = first_point_ + t * r1;

  return true;
}

double LineSegment::GetClosestPoints(const LineSegment& segment, Eigen::Vector3d& point,
                                     Eigen::Vector3d& other_point) const {
  auto r0 = second_point_ - first_point_;
  auto r1 = segment.GetSecondPoint() - segment.GetFirstPoint();
  auto a = segment.GetFirstPoint() - first_point_;

  // TODO What happens when the denominator is zero?
  double t0 = ((r0.dot(r1)) * (r1.dot(a)) - (r1.dot(r1)) * (r0.dot(a))) /
              ((r0.dot(r1)) * (r0.dot(r1)) - (r1.dot(r1)) * (r0.dot(r0)));
  double t1 = (t0 * (r0.dot(r0)) - (r0.dot(a))) / (r0.dot(r1));

  if (t0 > 1.0)
    t0 = 1.0;
  else if (t0 < 0)
    t0 = 0.0;

  if (t1 > 1.0)
    t1 = 1.0;
  else if (t1 < 0)
    t1 = 0.0;

  point = first_point_ + t0 * r0;
  other_point = segment.GetFirstPoint() + t1 * r1;

  return (point - other_point).norm();
}

Eigen::Vector3d LineSegment::GetClosestPoint(const Eigen::Vector3d& point) const {
  auto r = second_point_ - first_point_;
  double t = -(first_point_ - point).dot(r) / r.dot(r);
  if (t > 1.0)
    t = 1.0;
  else if (t < 0)
    t = 0.0;

  return (first_point_ + t * r);
}

}  // namespace discrepancy_planner
