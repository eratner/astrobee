// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/collision_object.h>
#include <string>

namespace ellis_planner {

CollisionObject::CollisionObject(const std::string& name) : name_(name) {}

CollisionObject::~CollisionObject() {}

const std::string& CollisionObject::GetName() const { return name_; }

visualization_msgs::Marker CollisionObject::GetMarker() const {
  visualization_msgs::Marker msg;
  return msg;
}

}  // namespace ellis_planner
