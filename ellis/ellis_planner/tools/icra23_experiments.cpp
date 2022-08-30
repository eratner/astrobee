// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ros/ros.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <ff_util/config_client.h>
#include <ff_msgs/MotionAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <std_srvs/Trigger.h>
#include <boost/circular_buffer.hpp>
#include <ellis_planner/ReportExecutionError.h>
#include <ellis_planner/AddObstacle.h>
#include <ellis_planner/ControlHistory.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <string>
#include <vector>

class Experiments {
 public:
  enum Method { OURS = 0, REPLANNING, CMAX, NUM_METHODS };

  enum State { REPLAN_NEEDED = 0, EXECUTING, DONE, ERROR };

  struct Output {
    Method method_;
    geometry_msgs::Pose start_pose_;
    double time_on_task_;
    bool reached_goal_;

    YAML::Node ToYaml() const {
      YAML::Node node;

      // TODO(eratner) Implement me

      return node;
    }
  };

  Experiments()
      : p_nh_("~"), tf_listener_(tf_buffer_), cfg_(&nh_, NODE_CHOREOGRAPHER), control_feedback_history_(100) {}

  ~Experiments() {}

  static std::string ToString(Method method, bool lowercase = true) {
    switch (method) {
      case OURS:
        return (lowercase ? "ours" : "OURS");
      case REPLANNING:
        return (lowercase ? "replanning" : "REPLANNING");
      case CMAX:
        return (lowercase ? "cmax" : "CMAX");
      default:
        break;
    }

    return (lowercase ? "unknown" : "UNKNOWN");
  }

  bool Init() {
    // Connect to the mobility server.
    mobility_client_.SetConnectedTimeout(30.0);
    mobility_client_.SetActiveTimeout(30.0);
    mobility_client_.SetResponseTimeout(30.0);
    mobility_client_.SetFeedbackCallback(std::bind(&Experiments::FeedbackCallback, this, std::placeholders::_1));
    mobility_client_.SetResultCallback(
      std::bind(&Experiments::ResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    mobility_client_.Create(&nh_, ACTION_MOBILITY_MOTION);
    while (ros::ok()) {
      if (mobility_client_.IsConnected()) {
        ROS_INFO("[Experiments] Mobility client connected to action server!");
        break;
      } else {
        ROS_WARN("[Experiments] Mobility client waiting to connect...");
      }

      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }

    add_obstacle_client_ = nh_.serviceClient<ellis_planner::AddObstacle>("/mob/ellis_planner/add_obstacle");
    clear_obstacles_client_ = nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/clear_obstacles");
    report_execution_error_client_ =
      nh_.serviceClient<ellis_planner::ReportExecutionError>("/mob/ellis_planner/report_execution_error");
    clear_execution_errors_client_ = nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/clear_execution_errors");

    while (ros::ok()) {
      if (report_execution_error_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for report exec error server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (clear_execution_errors_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for clear exec errors server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (add_obstacle_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for add obstacle server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (clear_obstacles_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for clear obstacles server...");
        ros::spinOnce();
      }
    }

    control_history_pub_ = nh_.advertise<ellis_planner::ControlHistory>("/exp/control_history", 10);

    subs_ = {nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1,
                                                [this](const tf2_msgs::TFMessageConstPtr& msg) {
                                                  if (bag_.isOpen()) bag_.write("/tf", ros::Time::now(), *msg);
                                                }),
             nh_.subscribe<tf2_msgs::TFMessage>("/tf_static", 1,
                                                [this](const tf2_msgs::TFMessageConstPtr& msg) {
                                                  if (bag_.isOpen()) bag_.write("/tf_static", ros::Time::now(), *msg);
                                                }),
             nh_.subscribe<visualization_msgs::Marker>("/mob/ellis_planner/vis", 1,
                                                       [this](const visualization_msgs::MarkerConstPtr& msg) {
                                                         if (bag_.isOpen())
                                                           bag_.write("/mob/ellis_planner/vis", ros::Time::now(), *msg);
                                                       }),
             nh_.subscribe<ellis_planner::ControlHistory>(
               "/exp/control_history", 1, [this](const ellis_planner::ControlHistoryConstPtr& msg) {
                 if (bag_.isOpen()) bag_.write("/exp/control_history", ros::Time::now(), *msg);
               })};

    return true;
  }

  bool Run() {
    // Load the experiment parameters.
    int num_experiments = p_nh_.param<int>("num_experiments", 0);
    int random_seed = p_nh_.param<int>("random_seed", 0);
    std::string output_dir = p_nh_.param<std::string>("output_dir", "/tmp");
    double time_limit_sec = p_nh_.param<double>("time_limit_sec", 1000.0);
    // TODO(eratner) Print info for debugging...

    double start_x = p_nh_.param<double>("start/pose/x", 0);
    double start_y = p_nh_.param<double>("start/pose/y", 0);
    double start_yaw = p_nh_.param<double>("start/pose/yaw", 0);
    bool randomize_start = p_nh_.param<bool>("start/randomize", false);

    double goal_x = p_nh_.param<double>("goal/pose/x", 0);
    double goal_y = p_nh_.param<double>("goal/pose/y", 0);
    double goal_yaw = p_nh_.param<double>("goal/pose/yaw", 0);
    bool randomize_goal = p_nh_.param<bool>("goal/randomize", false);

    goal_x_ = goal_x;
    goal_y_ = goal_y;

    // TODO(eratner) Get more parameters...

    for (unsigned int i = 0; i < num_experiments; ++i) {
      // Get the robot's start pose for this experiment.
      double exp_start_x = start_x;
      double exp_start_y = start_y;
      double exp_start_yaw = start_yaw;
      if (p_nh_.param<bool>("start/randomize", false)) {
        ROS_WARN("Randomize start not implemented!");
        // TODO(eratner) Implement me
      }

      double exp_goal_x = goal_x;
      double exp_goal_y = goal_y;
      double exp_goal_yaw = goal_yaw;
      if (p_nh_.param<bool>("goal/randomize", false)) {
        ROS_WARN("Randomize goal not implemented!");
        // TODO(eratner) Implement me
      }

      for (unsigned int j = 0; j < NUM_METHODS; ++j) {
        auto method = static_cast<Method>(j);
        bool run_method = p_nh_.param<bool>("run_method/" + ToString(method), true);
        if (!run_method) {
          ROS_WARN_STREAM("[Experiments] Skipping " << ToString(method) << "...");
          continue;
        }

        // Move robot to the start pose.
        cfg_.Set<std::string>("planner", "trapezoidal");
        if (!cfg_.Reconfigure()) {
          ROS_ERROR("[Experiments] Could not change to trapezoidal planner!");
          continue;
        }
        if (!MoveTo(exp_start_x, exp_start_y, exp_start_yaw)) {
          ROS_ERROR_STREAM("[Experiments] Could not move to experiment start (" << exp_start_x << ", " << exp_start_y
                                                                                << ", " << exp_start_yaw << ")!");
          continue;
        }
        cfg_.Set<std::string>("planner", "ellis");
        if (!cfg_.Reconfigure()) {
          ROS_ERROR("[Experiments] Could not change to ellis planner!");
          continue;
        }

        // TODO(eratner) Reset everything in the planner
        // TODO(eratner) Set the obstacles up in the planner
        // TODO(eratner) Set the disturbances/unknown obstacles up (how to do?)

        // Start writing to a bagfile.
        std::string path_to_bag = output_dir;
        if (path_to_bag.back() != '/') path_to_bag += '/';
        path_to_bag += ("exp_" + std::to_string(i) + "_" + ToString(method) + ".bag");
        bag_.open(path_to_bag, rosbag::bagmode::Write);

        // TODO(eratner) Run experiment while recording total time
        state_ = REPLAN_NEEDED;
        ros::Rate rate(5.0);
        while (ros::ok()) {
          switch (state_) {
            case REPLAN_NEEDED: {
              // Request to (re)plan and execute the path.
              if (!MoveTo(exp_goal_x, exp_goal_y, exp_goal_yaw)) {
                ROS_ERROR_STREAM("[Experiments] Move to (" << exp_goal_x << ", " << exp_goal_y << ", " << exp_goal_yaw
                                                           << ") failed!");
              } else {
                // Okay, now wait while the robot executes.
                state_ = EXECUTING;
              }
              break;
            }
            case EXECUTING: {
              // Robot is executing.
              break;
            }
            case DONE: {
              ROS_INFO("[Experiments] Done!");
              break;
            }
            case ERROR: {
              ROS_ERROR("[Experiments] An error occured!");
              break;
            }
            default:
              break;
          }

          if (state_ == ERROR) break;

          ros::spinOnce();
          rate.sleep();
        }

        // TODO(eratner) Check if AtGoal() to determine whether success

        // Stop writing to a bagfile.
        bag_.close();
      }
    }

    return true;
  }

 private:
  void ResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
        ROS_ERROR("[Experiments] Timeout on connect");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
        ROS_ERROR("[Experiments] Timeout on active");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
        ROS_ERROR("[Experiments] Timeout on response");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
        ROS_ERROR("[Experiments] Timeout on deadline");
        break;
      case ff_util::FreeFlyerActionState::Enum::SUCCESS:
      case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
      case ff_util::FreeFlyerActionState::Enum::ABORTED: {
        ROS_INFO_STREAM("[Experiments] Result: " << result->fsm_result);
        switch (result->response) {
          case ff_msgs::MotionResult::ALREADY_THERE:
          case ff_msgs::MotionResult::SUCCESS:
          case ff_msgs::MotionResult::PREEMPTED: {
            if (AtGoal()) {
              // Experiment is done.
              ROS_INFO("[Experiments] Experiment is done!");
              state_ = DONE;
            } else {
              ROS_WARN("[Experiments] Not done yet! Replanning...");
              state_ = REPLAN_NEEDED;
            }
            break;
          }
            // TODO(eratner) What is the difference between POSITION_ENDPOINT and POSITION?
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION_ENDPOINT:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_ATTITUDE:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION: {
            // A execution failure occurred!
            ROS_WARN("[Experiments] Something unexpected occurred!");

            ROS_INFO_STREAM("[Experiments] Publishing control history with " << control_history_.actual_pose.size()
                                                                             << " poses...");
            control_history_pub_.publish(control_history_);

            if (ReportExecutionError()) {
              ROS_INFO("[Experiments] Reported execution error...");
              state_ = REPLAN_NEEDED;
            } else {
              ROS_ERROR("[Experiments] Failed to report an execution error!");
              state_ = ERROR;
            }

            break;
          }
          default:
            break;
        }
      }
      default:
        break;
    }
  }

  void FeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
    double actual_x = 0.0, actual_y = 0.0, actual_z = 0.0, actual_yaw = 0.0;
    if (!GetPose(actual_x, actual_y, actual_z, actual_yaw)) {
      ROS_ERROR("    Failed to get robot's current pose!");
    } else {
      double err_pos = std::sqrt(std::pow(feedback->progress.setpoint.pose.position.x, 2) +
                                 std::pow(feedback->progress.setpoint.pose.position.y, 2));
      ROS_INFO_STREAM("    Actual pos: (" << actual_x << ", " << actual_y << ", " << actual_z
                                          << "), yaw: " << actual_yaw << ", err pos: " << err_pos);
    }

    control_history_.time.push_back(feedback->progress.setpoint.when);
    control_history_.desired_pose.push_back(feedback->progress.setpoint.pose);
    geometry_msgs::Pose actual_pose;
    actual_pose.position.x = actual_x;
    actual_pose.position.y = actual_y;
    actual_pose.position.z = actual_z;
    tf2::Quaternion actual_orien;
    actual_orien.setRPY(0, 0, actual_yaw);
    actual_pose.orientation.x = actual_orien.x();
    actual_pose.orientation.y = actual_orien.y();
    actual_pose.orientation.z = actual_orien.z();
    actual_pose.orientation.w = actual_orien.w();
    control_history_.actual_pose.push_back(actual_pose);

    control_feedback_history_.push_back(feedback->progress);
  }

  bool GetPose(double& x, double& y, double& z, double& yaw) {
    geometry_msgs::TransformStamped world_to_body;
    try {
      world_to_body =
        tf_buffer_.lookupTransform(std::string(FRAME_NAME_WORLD), std::string(FRAME_NAME_BODY), ros::Time(0));
    } catch (const tf2::TransformException& ex) {
      ROS_ERROR_STREAM("[Experiments] Error: " << ex.what());
      return false;
    }

    x = world_to_body.transform.translation.x;
    y = world_to_body.transform.translation.y;
    z = world_to_body.transform.translation.z;
    yaw = tf2::getYaw(world_to_body.transform.rotation);
    return true;
  }

  bool AtGoal(double goal_dist_tol = 0.15) {
    double curr_x = 0.0, curr_y = 0.0, curr_z = 0.0, curr_yaw = 0.0;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_WARN_STREAM("[Experiments] Could not check if at goal, because could not get current pose!");
      return false;
    }

    double dist = std::sqrt(std::pow(goal_x_ - curr_x, 2) + std::pow(goal_y_ - curr_y, 2));
    return (dist < goal_dist_tol);
  }

  bool ReportExecutionError() {
    int index = -1;
    for (int i = control_feedback_history_.size() - 1; i >= 0; --i) {
      const auto& fb = control_feedback_history_[i];
      if (fb.error_position < 0.05 && fb.error_attitude < 0.25) {
        index = i;
        break;
      }
    }

    if (index < 0) {
      ROS_ERROR("[Experiments] Failed to find root of error!");
      return false;
    }

    ROS_WARN_STREAM("[Experiments] Found root of error at index " << index << " (history size is "
                                                                  << control_feedback_history_.size() << ")");
    const auto& fb = control_feedback_history_[index];
    ROS_WARN_STREAM("[Experiments]   pos err: "
                    << fb.error_position << ", att err: " << fb.error_attitude << ", x: " << fb.setpoint.pose.position.x
                    << ", y: " << fb.setpoint.pose.position.y << ", yaw: " << tf2::getYaw(fb.setpoint.pose.orientation)
                    << ", vel x: " << fb.setpoint.twist.linear.x << ", vel y: " << fb.setpoint.twist.linear.y
                    << ", vel yaw: " << fb.setpoint.twist.angular.z);

    ellis_planner::ReportExecutionError report;
    report.request.x = fb.setpoint.pose.position.x;
    report.request.y = fb.setpoint.pose.position.y;
    report.request.yaw = tf2::getYaw(fb.setpoint.pose.orientation);

    // Find the action that the robot was executing.
    double n = std::sqrt(std::pow(fb.setpoint.twist.linear.x, 2) + std::pow(fb.setpoint.twist.linear.y, 2));
    if (std::abs(n) < 1e-6) {
      report.request.action_dir_x = 0.0;
      report.request.action_dir_y = 0.0;
    } else {
      report.request.action_dir_x = fb.setpoint.twist.linear.x / n;
      report.request.action_dir_y = fb.setpoint.twist.linear.y / n;
    }

    report.request.action_dir_yaw = 0.0;
    if (std::abs(fb.setpoint.twist.angular.z) > 1e-6) {
      if (fb.setpoint.twist.angular.z > 0.0)
        report.request.action_dir_yaw = 1.0;
      else
        report.request.action_dir_yaw = -1.0;
    }

    report.request.control_history = control_history_;

    // Report the execution error.
    if (!report_execution_error_client_.call(report)) {
      ROS_ERROR("[Experiments] Failed to call service to report an execution error");
      return false;
    }

    return true;
  }

  bool MoveTo(double x, double y, double yaw = 0.0) {
    // Get the robot's current pose.
    double curr_x, curr_y, curr_z, curr_yaw;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_ERROR("[ExperimentManager] Failed to get robot's current pose!");
      return false;
    }

    control_feedback_history_.clear();

    control_history_.desired_pose.clear();
    control_history_.actual_pose.clear();

    ff_msgs::MotionGoal goal;
    goal.flight_mode = "nominal";
    goal.command = ff_msgs::MotionGoal::MOVE;
    geometry_msgs::PoseStamped state;
    state.header.stamp = ros::Time::now();
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = curr_z;

    tf2::Quaternion orien;
    orien.setRPY(0, 0, yaw);

    state.pose.orientation.x = orien.x();
    state.pose.orientation.y = orien.y();
    state.pose.orientation.z = orien.z();
    state.pose.orientation.w = orien.w();
    goal.states = {state};
    ROS_INFO_STREAM("[Experiments] Sending motion goal with position ("
                    << state.pose.position.x << ", " << state.pose.position.y << ", " << state.pose.position.z
                    << ") and yaw " << yaw << "...");
    if (!mobility_client_.SendGoal(goal)) {
      ROS_ERROR("[Experiments] Mobility client did not accept goal");
      return false;
    }

    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle p_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> mobility_client_;
  ff_util::ConfigClient cfg_;
  ros::ServiceClient add_obstacle_client_;
  ros::ServiceClient clear_obstacles_client_;
  ros::ServiceClient report_execution_error_client_;
  ros::ServiceClient clear_execution_errors_client_;
  boost::circular_buffer<ff_msgs::ControlFeedback> control_feedback_history_;
  ellis_planner::ControlHistory control_history_;
  ros::Publisher control_history_pub_;
  State state_;

  // Experiment instance info.
  double goal_x_;
  double goal_y_;

  rosbag::Bag bag_;
  std::vector<ros::Subscriber> subs_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "icra23_experiments", ros::init_options::AnonymousName);

  Experiments exp;
  if (!exp.Init()) {
    ROS_ERROR("[Experiments] Failed to initialize!");
    return -1;
  }

  return exp.Run();
}
