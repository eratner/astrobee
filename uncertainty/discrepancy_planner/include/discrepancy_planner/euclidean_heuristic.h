#ifndef DISCREPANCY_PLANNER_EUCLIDEAN_HEURISTIC_H
#define DISCREPANCY_PLANNER_EUCLIDEAN_HEURISTIC_H

#include <discrepancy_planner/free_flyer_state_space.h>
#include <ellis_util/search/heuristic_search.h>
using ellis_util::search::Heuristic;

namespace discrepancy_planner {

class EuclideanHeuristic : public Heuristic<FreeFlyerStateSpace::StateDim> {
 public:
  typedef ellis_util::search::State<FreeFlyerStateSpace::StateDim> State;
  typedef typename StateSpace<FreeFlyerStateSpace::StateDim>::CostType CostType;

  EuclideanHeuristic(FreeFlyerStateSpace* state_space, double cost_per_meter);

  CostType Value(State* state);

 private:
  FreeFlyerStateSpace* state_space_;
  double cost_per_meter_;
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_EUCLIDEAN_HEURISTIC_H
