// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_SEARCH_H_
#define ELLIS_PLANNER_SEARCH_H_

#include <ellis_planner/environment.h>
#include <ellis_planner/clock.h>
#include <set>
#include <tuple>
#include <algorithm>
#include <vector>

namespace ellis_planner {

class Search {
 public:
  struct Profiling {
    Profiling();

    void Reset();

    int num_expansions_;
    double planning_time_sec_;
  };

  explicit Search(Environment* env);

  ~Search();

  bool Run(State::Ptr start_state, std::vector<State::Ptr>& path, double& path_cost);

  const Profiling& GetProfiling() const;

 private:
  void ReconstructPath(const State::Ptr goal_state, std::vector<State::Ptr>& path) const;

  Environment* env_;
  Profiling prof_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_SEARCH_H_
