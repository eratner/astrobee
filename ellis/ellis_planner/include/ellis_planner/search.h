// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_SEARCH_H_
#define ELLIS_PLANNER_SEARCH_H_

#include <ellis_planner/environment.h>
#include <set>
#include <tuple>
#include <algorithm>
#include <vector>
#include <chrono>

namespace ellis_planner {

class Search {
 public:
  struct Performance {
    Performance();

    void Reset();

    void StartClock();

    void StopClock();

    int num_expansions_;
    double planning_time_sec_;
    std::chrono::time_point<std::chrono::high_resolution_clock> planning_start_time_;
  };

  explicit Search(Environment* env);

  ~Search();

  bool Run(State::Ptr start_state, std::vector<State::Ptr>& path);

  const Performance& GetPerformance() const;

 private:
  void ReconstructPath(const State::Ptr goal_state, std::vector<State::Ptr>& path) const;

  Environment* env_;
  Performance perf_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_SEARCH_H_
