#include <discrepancy_planner/box.h>

namespace discrepancy_planner {

Box::Box(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
         double size_x, double size_y, double size_z)
    : position_(position),
      orientation_(orientation),
      size_x_(size_x),
      size_y_(size_y),
      size_z_(size_z) {}

const Eigen::Vector3d& Box::GetPosition() const { return position_; }

void Box::SetPosition(const Eigen::Vector3d& position) { position_ = position; }

const Eigen::Quaterniond& Box::GetOrientation() const { return orientation_; }

void Box::SetOrientation(const Eigen::Quaterniond& orientation) {
  orientation_ = orientation;
}

void Box::GetOrientationEuler(double& roll, double& pitch, double& yaw) const {
  QuaternionToRPY(orientation_.x(), orientation_.y(), orientation_.z(),
                  orientation_.w(), roll, pitch, yaw);
}

void Box::SetOrientationEuler(double roll, double pitch, double yaw) {
  orientation_ = RPYToQuaternion(roll, pitch, yaw);
}

double Box::GetSizeX() const { return size_x_; }

double Box::GetSizeY() const { return size_y_; }

double Box::GetSizeZ() const { return size_z_; }

std::array<Eigen::Vector3d, 8> Box::GetCorners() const {
  std::array<Eigen::Vector3d, 8> corners;

  // Back bottom left.
  corners[0] << -0.5 * size_x_, 0.5 * size_y_, -0.5 * size_z_;
  // Back bottom right.
  corners[1] << -0.5 * size_x_, -0.5 * size_y_, -0.5 * size_z_;
  // Back top right.
  corners[2] << -0.5 * size_x_, -0.5 * size_y_, 0.5 * size_z_;
  // Back top left.
  corners[3] << -0.5 * size_x_, 0.5 * size_y_, 0.5 * size_z_;

  // Front top left.
  corners[4] << 0.5 * size_x_, 0.5 * size_y_, 0.5 * size_z_;
  // Front top right.
  corners[5] << 0.5 * size_x_, -0.5 * size_y_, 0.5 * size_z_;
  // Front bottom right.
  corners[6] << 0.5 * size_x_, -0.5 * size_y_, -0.5 * size_z_;
  // Front bottom left.
  corners[7] << 0.5 * size_x_, 0.5 * size_y_, -0.5 * size_z_;

  // Transform each of the corners into global coordinates.
  for (auto& corner : corners)
    corner = orientation_.toRotationMatrix() * corner + position_;

  return corners;
}

std::array<LineSegment, 12> Box::GetEdges() const {
  std::array<LineSegment, 12> edges;

  auto corners = GetCorners();

  // Back face.
  edges[0] = LineSegment(corners[0], corners[1]);
  edges[1] = LineSegment(corners[1], corners[2]);
  edges[2] = LineSegment(corners[2], corners[3]);
  edges[3] = LineSegment(corners[3], corners[0]);

  // Front face.
  edges[4] = LineSegment(corners[4], corners[5]);
  edges[5] = LineSegment(corners[5], corners[6]);
  edges[6] = LineSegment(corners[6], corners[7]);
  edges[7] = LineSegment(corners[7], corners[4]);

  // Left / right sides.
  edges[8] = LineSegment(corners[3], corners[4]);  // Top left back to front.
  edges[9] = LineSegment(corners[0], corners[7]);  // Bottom left back to front.
  edges[10] = LineSegment(corners[2], corners[5]);  // Top right back to front.
  edges[11] =
    LineSegment(corners[1], corners[6]);  // Bottom right back to front.

  return edges;
}

bool Box::Intersects(const Box& other, Eigen::Vector3d* min_translation_vec,
                     std::array<Eigen::Vector3d, 2>* basis,
                     std::vector<Eigen::Vector3d>* points, bool verbose) const {
  auto corners = GetCorners();
  auto other_corners = other.GetCorners();

  auto rot = orientation_.toRotationMatrix();
  auto other_rot = other.GetOrientation().toRotationMatrix();

  const auto& unit_x = rot.col(0);
  const auto& unit_y = rot.col(1);
  const auto& unit_z = rot.col(2);

  const auto& other_unit_x = other_rot.col(0);
  const auto& other_unit_y = other_rot.col(1);
  const auto& other_unit_z = other_rot.col(2);

  // List of the possible separating axes.
  std::array<Eigen::Vector3d, 15> axes = {unit_x,
                                          unit_y,
                                          unit_z,
                                          other_unit_x,
                                          other_unit_y,
                                          other_unit_z,
                                          unit_x.cross(other_unit_x),
                                          unit_x.cross(other_unit_y),
                                          unit_x.cross(other_unit_z),
                                          unit_y.cross(other_unit_x),
                                          unit_y.cross(other_unit_y),
                                          unit_y.cross(other_unit_z),
                                          unit_z.cross(other_unit_x),
                                          unit_z.cross(other_unit_y),
                                          unit_z.cross(other_unit_z)};

  for (auto& axis : axes) {
    if (axis.norm() > 1e-9) axis.normalize();
  }

  std::array<std::array<Eigen::Vector3d, 2>, 15> bases;
  bases[0] = {unit_y, unit_z};
  bases[1] = {unit_x, unit_z};
  bases[2] = {unit_x, unit_y};
  bases[3] = {other_unit_y, other_unit_z};
  bases[4] = {other_unit_x, other_unit_z};
  bases[5] = {other_unit_x, other_unit_y};
  bases[6] = {unit_x, other_unit_x};
  bases[7] = {unit_x, other_unit_y};
  bases[8] = {unit_x, other_unit_z};
  bases[9] = {unit_y, other_unit_x};
  bases[10] = {unit_y, other_unit_y};
  bases[11] = {unit_y, other_unit_z};
  bases[12] = {unit_z, other_unit_x};
  bases[13] = {unit_z, other_unit_y};
  bases[14] = {unit_z, other_unit_z};

  int idx = 0;

  double min_overlap = 1e9;
  int min_idx = -1;

  for (const auto& axis : axes) {
    if (verbose) {
      std::cout << "Axis " << idx << " has norm " << axis.norm() << std::endl;
    }

    if (axis.norm() <= 1e-9) {
      // Handle a zero length axis (can occur when there are parallel surfaces).
      continue;
    }

    // Project both boxes onto the axis.
    auto proj = Proj(corners, axis);
    auto other_proj = Proj(other_corners, axis);
    if (verbose) {
      std::cout << "Proj: (" << proj.first << ", " << proj.second << ")"
                << std::endl;
      std::cout << "Other proj: (" << other_proj.first << ", "
                << other_proj.second << ")" << std::endl;
    }

    double overlap = GetOverlap(proj, other_proj);
    if (verbose)
      std::cout << "Overlap on axis " << idx << " is " << overlap << std::endl;

    if (overlap <= 0) return false;

    if (overlap < min_overlap) {
      if (verbose) {
        std::cout << "** New min overlap for axis " << idx << " with overlap "
                  << overlap << std::endl;
      }
      min_overlap = overlap;
      min_idx = idx;
    }

    idx++;
  }

  if (min_translation_vec && min_idx >= 0) {
    // Assign the minimum translation vector.
    *min_translation_vec = min_overlap * axes[min_idx];

    if (basis) {
      *basis = bases[min_idx];
    }

    Eigen::Vector3d trans = other.GetPosition() - GetPosition();
    if (trans.dot(*min_translation_vec) < 0) {
      // Flip the direction of the vector.
      *min_translation_vec = -(*min_translation_vec);
    }

    std::cout << "MTV is axis " << min_idx << " with overlap " << min_overlap
              << std::endl;
  }

  // Find the contact manifold (i.e. set of points representing contact).
  if (points && min_idx >= 0) {
    if (0 <= min_idx && min_idx <= 5) {
      std::cout << "Face contact (multiple contacts)!" << std::endl;
      // TODO
    } else if (6 <= min_idx && min_idx <= 14) {
      std::cout << "Edge contact (single contact point)!" << std::endl;

      // TODO This is not very good.
      // Consider all pairs of edges and check for intersection (below a
      // threshold).
      auto edges = GetEdges();
      auto other_edges = other.GetEdges();

      double closest_dist = 1e9;
      Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();

      for (auto edge : edges) {
        for (auto other_edge : other_edges) {
          Eigen::Vector3d point = Eigen::Vector3d::Zero();
          Eigen::Vector3d other_point = Eigen::Vector3d::Zero();

          double dist = edge.GetClosestPoints(other_edge, point, other_point);
          if (dist < closest_dist) {
            closest_dist = dist;
            closest_point = point;
          }
        }
      }

      points->push_back(closest_point);
    }
  }

  return true;
}

std::pair<double, double> Box::Proj(
  const std::array<Eigen::Vector3d, 8>& corners,
  const Eigen::Vector3d& axis) const {
  std::pair<double, double> projection;
  projection.first = axis.dot(corners[0]);
  projection.second = projection.first;

  for (const auto& corner : corners) {
    double proj = axis.dot(corner);
    if (proj < projection.first)
      projection.first = proj;
    else if (proj > projection.second)
      projection.second = proj;
  }

  return projection;
}

double Box::GetOverlap(const std::pair<double, double>& seg,
                       const std::pair<double, double>& other_seg) const {
  return std::max(0., std::min(seg.second, other_seg.second) -
                        std::max(seg.first, other_seg.first));
}

}  // namespace discrepancy_planner
