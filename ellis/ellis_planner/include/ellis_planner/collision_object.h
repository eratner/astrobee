// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_COLLISION_OBJECT_H_
#define ELLIS_PLANNER_COLLISION_OBJECT_H_

#include <visualization_msgs/Marker.h>
#include <string>

namespace ellis_planner {

class CollisionObject {
 public:
  typedef CollisionObject* Ptr;

  explicit CollisionObject(const std::string& name);

  virtual ~CollisionObject();

  virtual bool InCollision(const CollisionObject::Ptr other) const = 0;

  const std::string& GetName() const;

  virtual visualization_msgs::Marker GetMarker() const;

 private:
  std::string name_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_COLLISION_OBJECT_H_
