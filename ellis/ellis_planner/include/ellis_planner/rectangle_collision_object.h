// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_RECTANGLE_COLLISION_OBJECT_H_
#define ELLIS_PLANNER_RECTANGLE_COLLISION_OBJECT_H_

#include <ellis_planner/collision_object.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <string>

namespace ellis_planner {

class RectangleCollisionObject : public CollisionObject {
 public:
  typedef RectangleCollisionObject* Ptr;
  typedef std::array<double, 2> Point;

  RectangleCollisionObject(const std::string& name, double x, double y, double yaw, double size_x, double size_y);

  ~RectangleCollisionObject();

  bool InCollision(const CollisionObject::Ptr other) const;

  visualization_msgs::Marker GetMarker() const;

  std::array<Eigen::Vector2d, 4> GetVertices() const;

  bool Contains(const Eigen::Vector2d& point) const;

  double GetX() const;

  void SetX(double x);

  double GetY() const;

  void SetY(double y);

  double GetYaw() const;

  void SetYaw(double yaw);

  double GetSizeX() const;

  void SetSizeX(double size_x);

  double GetSizeY() const;

  void SetSizeY(double size_y);

 private:
  double x_;
  double y_;
  double yaw_;

  double size_x_;
  double size_y_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_RECTANGLE_COLLISION_OBJECT_H_
