// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/rectangle_collision_object.h>
#include <string>

namespace ellis_planner {

RectangleCollisionObject::RectangleCollisionObject(const std::string& name, double x, double y, double yaw,
                                                   double size_x, double size_y)
    : CollisionObject(name), x_(x), y_(y), yaw_(yaw), size_x_(size_x), size_y_(size_y) {}

RectangleCollisionObject::~RectangleCollisionObject() {}

bool RectangleCollisionObject::InCollision(const CollisionObject::Ptr other) const {
  auto other_rect = dynamic_cast<RectangleCollisionObject::Ptr>(other);
  if (other_rect) {
    // Check if any vertex of this rectangle is contained in the other rectangle.
    auto vertices = GetVertices();
    for (const auto& vertex : vertices) {
      if (other_rect->Contains(vertex)) return true;
    }

    // Check if any vertex of the other rectangle is contained in this rectangle.
    auto other_vertices = other_rect->GetVertices();
    for (const auto& vertex : other_vertices) {
      if (Contains(vertex)) return true;
    }

    // Check if any edge of this rectangle intersects with any edge of the other rectangle.
    // TODO(eratner) Implement me
  }
  return false;
}

visualization_msgs::Marker RectangleCollisionObject::GetMarker() const {
  visualization_msgs::Marker msg;
  msg.header.frame_id = "";  // TODO(eratner) Should be filled in by caller
  msg.ns = "/mob/ellis_planner/collision_rect/" + this->GetName();
  msg.id = 0;
  msg.type = visualization_msgs::Marker::CUBE;
  msg.pose.position.x = x_;
  msg.pose.position.y = y_;
  msg.pose.position.z = -0.674614;  // TODO(eratner) Fix this
  tf2::Quaternion orien;
  orien.setRPY(0.0, 0.0, yaw_);
  msg.pose.orientation.x = orien.x();
  msg.pose.orientation.y = orien.y();
  msg.pose.orientation.z = orien.z();
  msg.pose.orientation.w = orien.w();
  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 0.5;
  msg.scale.x = size_x_;
  msg.scale.y = size_y_;
  msg.scale.z = 0.5;  // TODO(eratner) Fix this
  return msg;
}

std::array<Eigen::Vector2d, 4> RectangleCollisionObject::GetVertices() const {
  std::array<Eigen::Vector2d, 4> vertices = {
    Eigen::Vector2d(0.5 * size_x_, 0.5 * size_y_), Eigen::Vector2d(-0.5 * size_x_, 0.5 * size_y_),
    Eigen::Vector2d(-0.5 * size_x_, -0.5 * size_y_), Eigen::Vector2d(0.5 * size_x_, -0.5 * size_y_)};

  Eigen::Matrix<double, 2, 2> rot = Eigen::Matrix<double, 2, 2>::Identity();
  rot(0, 0) = std::cos(yaw_);
  rot(0, 1) = -std::sin(yaw_);
  rot(1, 0) = std::sin(yaw_);
  rot(1, 1) = std::cos(yaw_);
  Eigen::Vector2d trans(x_, y_);

  for (int i = 0; i < 4; ++i) vertices[i] = trans + rot * vertices[i];

  return vertices;
}

bool RectangleCollisionObject::Contains(const Eigen::Vector2d& point) const {
  auto vertices = GetVertices();
  for (int i = 0; i < 4; ++i) {
    const auto& first_vertex = vertices[i];
    const auto& second_vertex = vertices[(i + 1) % 4];
    Eigen::Vector2d first_vertex_to_point = point - first_vertex;
    Eigen::Vector2d edge = second_vertex - first_vertex;
    double cross_z = edge.x() * first_vertex_to_point.y() - first_vertex_to_point.x() * edge.y();
    if (cross_z < 0.0) return false;
  }
  return true;
}

double RectangleCollisionObject::GetX() const { return x_; }

void RectangleCollisionObject::SetX(double x) { x_ = x; }

double RectangleCollisionObject::GetY() const { return y_; }

void RectangleCollisionObject::SetY(double y) { y_ = y; }

double RectangleCollisionObject::GetYaw() const { return yaw_; }

void RectangleCollisionObject::SetYaw(double yaw) { yaw_ = yaw; }

}  // namespace ellis_planner
