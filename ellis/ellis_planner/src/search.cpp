// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/search.h>
#include <vector>
#include <set>

namespace ellis_planner {

Search::Profiling::Profiling() : num_expansions_(0), planning_time_sec_(0.0) {}

void Search::Profiling::Reset() {
  num_expansions_ = 0;
  planning_time_sec_ = 0.0;
}

Search::Search(Environment* env) : env_(env) {}

Search::~Search() {}

bool Search::Run(State::Ptr start_state, std::vector<State::Ptr>& path, double& path_cost) {
  prof_.Reset();
  env_->prof_.Reset();
  ScopedClock clock(&prof_.planning_time_sec_, false);

  path_cost = 1e9;

  std::set<std::tuple<double, unsigned int>> open;
  open.insert(std::make_tuple(env_->GetHeuristicCostToGoal(start_state), start_state->GetId()));

  start_state->cost_to_come_ = 0.0;
  start_state->parent_ = nullptr;

  const auto& actions = env_->GetActions();

  while (!open.empty()) {
    auto e = *open.begin();
    open.erase(open.begin());

    unsigned int state_id = std::get<1>(e);
    auto state = env_->GetState(state_id);
    if (env_->IsGoal(state)) {
      // Found the goal! Now, reconstruct the path.
      ReconstructPath(state, path);
      path_cost = state->cost_to_come_;
      return true;
    }

    prof_.num_expansions_++;

    for (const auto& action : actions) {
      auto outcome_and_cost = env_->GetOutcome(state, action);
      auto outcome_state = std::get<0>(outcome_and_cost);
      if (outcome_state) {
        auto cost = std::get<1>(outcome_and_cost);
        double cost_to_come_via = state->cost_to_come_ + cost;
        if (cost_to_come_via < outcome_state->cost_to_come_) {
          outcome_state->cost_to_come_ = cost_to_come_via;
          outcome_state->parent_ = state;
          open.insert(std::make_tuple(outcome_state->cost_to_come_ + env_->GetHeuristicCostToGoal(outcome_state),
                                      outcome_state->GetId()));
        }
      }
    }
  }

  // Planner must have failed to find a path.
  return false;
}

const Search::Profiling& Search::GetProfiling() const { return prof_; }

void Search::ReconstructPath(const State::Ptr goal_state, std::vector<State::Ptr>& path) const {
  State::Ptr state = goal_state;
  while (state) {
    path.push_back(state);
    state = state->parent_;
  }
  std::reverse(path.begin(), path.end());
}

}  // namespace ellis_planner
