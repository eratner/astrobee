// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ros/ros.h>
#include <ellis_planner/gp.h>
#include <iostream>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_gp", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  // TODO Publishers...

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

  ros::spin();
  return 0;
}
