// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ros/ros.h>
#include <ellis_planner/gp.h>
#include <ellis_planner/linear_dynamics.h>
#include <iostream>
#include <vector>

void Test1() {
  ellis_planner::GP<1>::Parameters params;
  params.v0_ = 0.01;
  params.v1_ = 1.0;
  params.weights_ = {1.0};

  ellis_planner::GP<1> model(params);
  model.Train(
    {ellis_planner::GP<1>::InputVec(1.0), ellis_planner::GP<1>::InputVec(3.0), ellis_planner::GP<1>::InputVec(4.0)},
    {0.2, -0.2, 0.8});

  double x_min = -5.1;
  double x_max = 5.1;
  double step_size = 0.1;

  double x_test = x_min;
  while (x_test <= x_max) {
    ellis_planner::GP<1>::InputVec x(x_test);
    double mean = model.MeanFunc(x);
    double std_dev = std::sqrt(model.VarFunc(x));
    Eigen::Matrix<double, 1, 1> deriv_mean = model.GetFirstDerivOfMeanFunc(x);
    Eigen::Matrix<double, 1, 1> dderiv_var = model.GetSecondDerivOfVarFunc(x);
    std::cout << "x: " << x << ", mean: " << mean << ", std dev: " << std_dev << ", deriv mean: " << deriv_mean
              << ", dderiv var: " << dderiv_var << std::endl;
    x_test += step_size;
  }
}

void Test2() {
  // Dynamics.
  const double time_step = 0.1;
  Eigen::Matrix<double, 1, 1> A = Eigen::Matrix<double, 1, 1>::Identity();
  Eigen::Matrix<double, 1, 1> B = time_step * Eigen::Matrix<double, 1, 1>::Identity();
  ellis_planner::LinearDynamics<1, 1> dyn(A, B);

  dyn.GetDisturbances()[0].GetParameters().v0_ = 0.0001;
  dyn.GetDisturbances()[0].GetParameters().v1_ = 0.0001;
  dyn.GetDisturbances()[0].GetParameters().weights_ = {0.2, 0.2};
  Eigen::Matrix<double, 2, 1> x_in;
  x_in << 0.65, 0.25;
  dyn.GetDisturbances()[0].Train({x_in}, {0.01});

  // Make a prediction.
  Eigen::Matrix<double, 1, 1> start_state;
  start_state << 0.0;

  const double vel = 0.25;
  std::vector<Eigen::Matrix<double, 1, 1>> controls(25, Eigen::Matrix<double, 1, 1>(vel));
  std::vector<Eigen::Matrix<double, 1, 1>> pred_mean;
  std::vector<Eigen::Matrix<double, 1, 1>> pred_cov;
  dyn.Predict(start_state, controls, pred_mean, pred_cov);
  for (int t = 0; t < pred_mean.size(); ++t) {
    std::cout << "t: " << t << ", mean: " << pred_mean[t] << ", cov: " << pred_cov[t] << std::endl;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_gp", ros::init_options::AnonymousName);

  ros::NodeHandle nh;

  Test1();
  Test2();

  ros::spin();
  return 0;
}
