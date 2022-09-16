// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_DISTURBANCE_MODEL_MANAGER_H_
#define ELLIS_PLANNER_DISTURBANCE_MODEL_MANAGER_H_

#include <ellis_planner/gp.h>
#include <extern/nanoflann.hpp>
#include <vector>
#include <utility>

namespace ellis_planner {

class DisturbanceModelManager {
 public:
  typedef GP<4> DisturbanceModel;

  DisturbanceModelManager();

  ~DisturbanceModelManager();

  void Clear();

  int DisturbanceModelIndexAt(double x, double y, double max_dist = 0.1) const;

  void TrainModels(const std::vector<Eigen::Matrix<double, 4, 1>>& training_inputs,
                   const std::vector<double>& training_targets_x, const std::vector<double>& training_targets_y);

  std::pair<DisturbanceModel*, DisturbanceModel*> GetDisturbanceModels(int index);

  int GetNumModels() const;

 private:
  struct Dataset {
    struct Point {
      explicit Point(double x = 0.0, double y = 0.0, int model_index = -1) : x_(x), y_(y), model_index_(model_index) {}

      double x_;
      double y_;
      int model_index_;
    };

    inline std::size_t kdtree_get_point_count() const { return points_.size(); }

    inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
      if (dim == 0)
        return points_[idx].x_;
      else
        return points_[idx].y_;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
      return false;
    }

    std::vector<Point> points_;
  };
  Dataset tree_data_;
  nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, Dataset>, Dataset, 2>* tree_;

  std::vector<DisturbanceModel*> disturbance_model_x_;
  std::vector<DisturbanceModel*> disturbance_model_y_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_DISTURBANCE_MODEL_MANAGER_H_
