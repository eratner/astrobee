#ifndef DISCREPANCY_PLANNER_RTAA_H
#define DISCREPANCY_PLANNER_RTAA_H

#include <ellis_util/search/heuristic_search.h>
using namespace ellis_util::search;

namespace discrepancy_planner {

// Real-Time Adaptive A* implementation.
template <unsigned int StateDim> class RTAA {
public:
  typedef PriorityQueue<typename StateSpace<StateDim>::CostType> OpenList;

  RTAA(StateSpace<StateDim> *state_space,
       Heuristic<StateDim> *heuristic = nullptr, int max_expansions = -1)
      : state_space_(state_space), heuristic_(heuristic),
        max_expansions_(max_expansions), update_heuristic_(true),
        last_path_cost_(StateSpace<StateDim>::INFINITE_COST) {}

  State<StateDim> *Search(State<StateDim> *start_state) {
    if (!start_state->GetSearchInfo()) {
      std::cout << "[RTAA::Search] Initializing search info for start state "
                << *start_state << std::endl;
      typename StateSpace<StateDim>::CostType heur = 0.0;
      if (heuristic_)
        heur = heuristic_->Value(start_state);

      start_state->SetSearchInfo(
          new HeuristicSearchInfo<StateDim>(0.0, heur, nullptr));
    } else {
      auto start_state_info =
          start_state->template GetSearchInfo<HeuristicSearchInfo>();
      start_state_info->cost_to_come_ = 0;
      start_state_info->parent_ = nullptr;
      start_state_info->parent_action_ = -1;
    }

    last_path_cost_ = StateSpace<StateDim>::INFINITE_COST;
    last_expansions_ = 0;
    last_expanded_state_ids_.clear();

    // Initialize empty open and closed lists.
    OpenList open;
    std::vector<State<StateDim> *> closed;

    OpenState(open, start_state);

    bool print_open_list = false;

    // int expansions = 0;
    State<StateDim> *last_state_expanded = nullptr;
    while (open.Size() > 0) {
      // Check if max expansions have already been done.
      // if (max_expansions_ > 0 && expansions >= max_expansions_) {
      if (max_expansions_ > 0 && last_expansions_ >= max_expansions_) {
        std::cout << "[RTAA::Search] Reached max expansion limit of "
                  << max_expansions_ << std::endl;
        break;
      }

      auto state = static_cast<State<StateDim> *>(open.Front());
      if (state->IsGoal()) {
        std::cout << "[RTAA::Search] Found goal after "
                  << last_expansions_ // expansions
                  << " expansions, with f = " << state->GetPriority() << "!"
                  << std::endl;
        // TODO DEBUGGING
        if (state->GetPriority() > 1000) {
          std::cout << "Something seems wrong! Open list: " << std::endl;
          print_open_list = true;
        }
        break;
      }

      open.Pop();

      // Expand the state.
      auto state_info = state->template GetSearchInfo<HeuristicSearchInfo>();
      state_info->closed_ = true;
      closed.push_back(state);
      // expansions++;
      last_expansions_++;
      last_state_expanded = state;
      last_expanded_state_ids_.push_back(state->GetId());

      auto actions = state_space_->GetActions(state);
      for (auto action : actions) {
        auto succs_and_costs = state_space_->GetSucc(state, action);

        auto succs = std::get<0>(succs_and_costs);
        auto costs = std::get<1>(succs_and_costs);

        if (succs.size() != 1) {
          // PRINT_WARN_STREAM("[RTAA::Search] For action "
          //                   << action << " at state " << *state << ", "
          //                   << succs.size() << " successors!");
          continue;
        }

        auto succ = succs[0];
        auto cost = costs[0];

        if (!succ->GetSearchInfo()) {
          // If the search has never seen this state before.
          typename StateSpace<StateDim>::CostType heur = 0.0;
          if (heuristic_)
            heur = heuristic_->Value(succ);

          succ->SetSearchInfo(new HeuristicSearchInfo<StateDim>(
              StateSpace<StateDim>::INFINITE_COST, heur, nullptr));
        }

        auto succ_info = succ->template GetSearchInfo<HeuristicSearchInfo>();
        if (succ_info->closed_)
          continue;

        if (state_info->cost_to_come_ + cost < succ_info->cost_to_come_) {
          // Found a better path through state to succ.
          succ_info->parent_ = state;
          succ_info->parent_action_ = action;
          succ_info->cost_to_come_ = state_info->cost_to_come_ + cost;
        }

        OpenState(open, succ);
      }
    }

    State<StateDim> *best_state = nullptr;
    if (open.Size() > 0) {
      // Get the best state found so far.
      best_state = static_cast<State<StateDim> *>(open.Front());
      std::cout << "[RTAA::Search] Reached best state " << *best_state
                << std::endl;
      auto best_state_info =
          best_state->template GetSearchInfo<HeuristicSearchInfo>();
      last_path_cost_ = best_state_info->cost_to_come_;
    } else {
      // Failure.
      std::cout << "[RTAA::Search] Failed to find a solution after "
                << last_expansions_ // expansions
                << " expansions!" << std::endl;
      if (last_state_expanded)
        std::cout << "[RTAA::Search] Open list is empty, so using last state "
                     "expanded "
                  << *last_state_expanded << std::endl;
      best_state = last_state_expanded;
      auto best_state_info =
          best_state->template GetSearchInfo<HeuristicSearchInfo>();
      last_path_cost_ = best_state_info->cost_to_come_;
    }

    // Update the heuristic.
    if (update_heuristic_)
      UpdateHeuristic(closed, best_state);

    // Do some clean up.
    for (auto state : closed) {
      auto state_info = state->template GetSearchInfo<HeuristicSearchInfo>();
      state_info->cost_to_come_ = StateSpace<StateDim>::INFINITE_COST;
      state_info->closed_ = false;
    }

    while (open.Size() > 0) {
      auto state = static_cast<State<StateDim> *>(open.Pop());
      auto state_info = state->template GetSearchInfo<HeuristicSearchInfo>();
      if (print_open_list) {
        std::cout << "  " << *state << ", g = " << state_info->cost_to_come_
                  << ", h = " << state_info->heur_cost_to_go_ << std::endl;
      }
      state_info->cost_to_come_ = StateSpace<StateDim>::INFINITE_COST;
      state_info->closed_ = false;
    }

    return best_state;
  }

  int GetMaxExpansions() const { return max_expansions_; }

  void SetMaxExpansions(int expansions) { max_expansions_ = expansions; }

  bool GetUpdateHeuristic() const { return update_heuristic_; }

  void SetUpdateHeuristic(bool update) { update_heuristic_ = update; }

  typename StateSpace<StateDim>::CostType GetLastPathCost() const {
    return last_path_cost_;
  }

  unsigned int GetLastExpansions() const { return last_expansions_; }

  const std::vector<unsigned int> &GetLastExpandedStateIds() const {
    return last_expanded_state_ids_;
  }

private:
  bool OpenState(OpenList &open, State<StateDim> *state) {
    auto state_info = state->template GetSearchInfo<HeuristicSearchInfo>();
    if (!state->InPriorityQueue()) {
      state->SetPriority(state_info->cost_to_come_ +
                         state_info->heur_cost_to_go_);
      open.Push(state);
    } else {
      // Already in the open list; priority must have decreased.
      if (!open.DecreasePriority(state, state_info->cost_to_come_ +
                                            state_info->heur_cost_to_go_)) {
        std::cout << "[RTAA::OpenState] An error occured!" << std::endl;
        return false;
      }
    }

    return true;
  }

  void UpdateHeuristic(const std::vector<State<StateDim> *> &closed,
                       State<StateDim> *best_state) {
    if (!best_state)
      return;

    auto best_state_info =
        best_state->template GetSearchInfo<HeuristicSearchInfo>();

    for (auto state : closed) {
      auto state_info = state->template GetSearchInfo<HeuristicSearchInfo>();
      state_info->heur_cost_to_go_ = best_state_info->cost_to_come_ +
                                     best_state_info->heur_cost_to_go_ -
                                     state_info->cost_to_come_;
    }
    std::cout << "[RTAA::UpdateHeuristic] Updated heuristic for "
              << closed.size() << " states" << std::endl;
  }

  StateSpace<StateDim> *state_space_;
  Heuristic<StateDim> *heuristic_;
  int max_expansions_;
  bool update_heuristic_;
  typename StateSpace<StateDim>::CostType last_path_cost_;
  unsigned int last_expansions_;
  std::vector<unsigned int> last_expanded_state_ids_;
};

} // namespace discrepancy_planner

#endif // DISCREPANCY_PLANNER_RTAA_H
