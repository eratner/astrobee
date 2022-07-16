// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_SEARCH_H_
#define ELLIS_PLANNER_SEARCH_H_

#include <ellis_planner/environment.h>
#include <set>
#include <tuple>
#include <algorithm>
#include <vector>

namespace ellis_planner {

class Search {
 public:
  explicit Search(Environment* env);

  ~Search();

  bool Run(State::Ptr start_state, std::vector<State::Ptr>& path);

 private:
  void ReconstructPath(const State::Ptr goal_state, std::vector<State::Ptr>& path) const;

  Environment* env_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_SEARCH_H_
