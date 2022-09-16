// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/disturbance_model_manager.h>
#include <vector>
#include <utility>

namespace ellis_planner {

DisturbanceModelManager::DisturbanceModelManager() : tree_(nullptr) {}

DisturbanceModelManager::~DisturbanceModelManager() { Clear(); }

void DisturbanceModelManager::Clear() {
  for (auto model : disturbance_model_x_) {
    if (model) delete model;
  }
  disturbance_model_x_.clear();

  for (auto model : disturbance_model_y_) {
    if (model) delete model;
  }
  disturbance_model_y_.clear();

  if (tree_) {
    delete tree_;
    tree_ = nullptr;
  }

  tree_data_.points_.clear();
}

int DisturbanceModelManager::DisturbanceModelIndexAt(double x, double y, double max_dist) const {
  if (!tree_ || tree_data_.points_.empty()) {
    // No disturbance model.
    return -1;
  }

  nanoflann::SearchParams params;
  nanoflann::KNNResultSet<double> result_set(1);
  std::size_t return_index;
  double out_dist_squared;
  result_set.init(&return_index, &out_dist_squared);
  double query_point[2] = {x, y};
  tree_->findNeighbors(result_set, query_point, params);
  if (std::sqrt(out_dist_squared) > max_dist) {
    // Disturbance model too far away.
    return -1;
  }

  return tree_data_.points_[return_index].model_index_;
}

void DisturbanceModelManager::TrainModels(const std::vector<Eigen::Matrix<double, 4, 1>>& training_inputs,
                                          const std::vector<double>& training_targets_x,
                                          const std::vector<double>& training_targets_y) {
  // TODO(eratner) Should we check if there is an existing model that the data should update before creating new models?
  int model_index = disturbance_model_x_.size();

  DisturbanceModel* model_x = new DisturbanceModel();
  // TODO(eratner) Don't hard-code the GP parameters here
  model_x->GetParameters().v0_ = 0.005;
  model_x->GetParameters().v1_ = 0.001;
  model_x->GetParameters().weights_ = {0.125, 0.125, 0.125, 0.125};
  model_x->Train(training_inputs, training_targets_x);
  disturbance_model_x_.push_back(model_x);

  DisturbanceModel* model_y = new DisturbanceModel();
  // TODO(eratner) Don't hard-code the GP parameters here
  model_y->GetParameters().v0_ = 0.005;
  model_y->GetParameters().v1_ = 0.001;
  model_y->GetParameters().weights_ = {0.125, 0.125, 0.125, 0.125};
  model_y->Train(training_inputs, training_targets_y);
  disturbance_model_y_.push_back(model_y);

  for (int i = 0; i < training_inputs.size(); ++i) {
    double disturbance_mag = std::sqrt(std::pow(training_targets_x[i], 2) + std::pow(training_targets_y[i], 2));
    if (disturbance_mag > 1e-4) {
      // Add only non-zero disturbances.
      tree_data_.points_.push_back(Dataset::Point(training_inputs[i](0), training_inputs[i](1), model_index));
    }
  }
  if (tree_) delete tree_;
  tree_ = new nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, Dataset>, Dataset, 2>(
    2, tree_data_, {10});
}

std::pair<DisturbanceModelManager::DisturbanceModel*, DisturbanceModelManager::DisturbanceModel*>
DisturbanceModelManager::GetDisturbanceModels(int index) {
  if (index < 0 || index >= disturbance_model_x_.size()) return std::make_pair(nullptr, nullptr);

  return std::make_pair(disturbance_model_x_[index], disturbance_model_y_[index]);
}

int DisturbanceModelManager::GetNumModels() const { return disturbance_model_x_.size(); }

}  // namespace ellis_planner
