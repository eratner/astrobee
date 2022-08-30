// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ros/ros.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <ff_util/config_client.h>
#include <ff_msgs/MotionAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <boost/circular_buffer.hpp>
#include <ellis_planner/ReportExecutionError.h>
#include <ellis_planner/AddObstacle.h>
#include <ellis_planner/ControlHistory.h>
#include <array>
#include <string>
#include <vector>

class ExperimentManager {
 public:
  ExperimentManager()
      : p_nh_("~"),
        tf_listener_(tf_buffer_),
        state_(READY),
        cfg_(&nh_, NODE_CHOREOGRAPHER),
        control_feedback_history_(100),
        waypoint_index_(0) {}

  ~ExperimentManager() {}

  enum State { READY = 0, MOVING, REPLAN_NEEDED, ERROR };

  bool Init() {
    // Connect to the mobility server.
    mobility_client_.SetConnectedTimeout(30.0);
    mobility_client_.SetActiveTimeout(30.0);
    mobility_client_.SetResponseTimeout(30.0);
    mobility_client_.SetFeedbackCallback(std::bind(&ExperimentManager::FeedbackCallback, this, std::placeholders::_1));
    mobility_client_.SetResultCallback(
      std::bind(&ExperimentManager::ResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    mobility_client_.Create(&nh_, ACTION_MOBILITY_MOTION);
    while (ros::ok()) {
      if (mobility_client_.IsConnected()) {
        ROS_INFO("[ExperimentManager] Mobility client connected to action server!");
        break;
      } else {
        ROS_WARN("[ExperimentManager] Mobility client waiting to connect...");
      }

      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }

    XmlRpc::XmlRpcValue waypoints;
    if (p_nh_.getParam("waypoints", waypoints)) {
      for (int i = 0; i < waypoints.size(); ++i) {
        XmlRpc::XmlRpcValue w = waypoints[i];
        double x = 0.0, y = 0.0, yaw = 0.0;
        std::string planner_type = "trapezoidal";
        for (auto it = w.begin(); it != w.end(); ++it) {
          if (it->first == "x")
            x = static_cast<double>(it->second);
          else if (it->first == "y")
            y = static_cast<double>(it->second);
          else if (it->first == "yaw")
            yaw = static_cast<double>(it->second);
          else if (it->first == "planner")
            planner_type = static_cast<std::string>(it->second);
        }
        ROS_INFO_STREAM("  " << i << ": (" << x << ", " << y << ", " << yaw << "), planner: " << planner_type);
        waypoint_.push_back({x, y, yaw});
        planner_type_.push_back(planner_type);
      }
    } else {
      ROS_WARN("[ExperimentManager] No waypoints specified");
    }

    add_obstacle_client_ = nh_.serviceClient<ellis_planner::AddObstacle>("/mob/ellis_planner/add_obstacle");
    clear_obstacles_client_ = nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/clear_obstacles");

    // report_execution_error_client_ =
    // nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/report_execution_error");
    report_execution_error_client_ =
      nh_.serviceClient<ellis_planner::ReportExecutionError>("/mob/ellis_planner/report_execution_error");
    clear_execution_errors_client_ = nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/clear_execution_errors");

    while (ros::ok()) {
      if (report_execution_error_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[ExperimentManager] Waiting for report exec error server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (clear_execution_errors_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[ExperimentManager] Waiting for clear exec errors server...");
        ros::spinOnce();
      }
    }

    stop_robot_on_execution_error_ = p_nh_.param<bool>("stop_robot_on_execution_error", true);
    ROS_INFO_STREAM("Stopping robot on execution failure? " << (stop_robot_on_execution_error_ ? "YES" : "NO"));

    while (ros::ok()) {
      if (add_obstacle_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[ExperimentManager] Waiting for add obstacle server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (clear_obstacles_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[ExperimentManager] Waiting for clear obstacles server...");
        ros::spinOnce();
      }
    }

    // Clear any existing obstacles.
    std_srvs::Trigger srv;
    if (clear_obstacles_client_.call(srv))
      ROS_INFO("[ExperimentManager] Clearing existing obstacles...");
    else
      ROS_WARN("[ExperimentManager] Failed to call service to clear existing obstacles!");

    XmlRpc::XmlRpcValue obstacles;
    if (p_nh_.getParam("obstacles", obstacles)) {
      for (int i = 0; i < obstacles.size(); ++i) {
        XmlRpc::XmlRpcValue o = obstacles[i];
        double x = 0.0, y = 0.0, yaw = 0.0;
        double size_x = 0.0, size_y = 0.0;
        std::string type = "rectangle";
        std::string name = "obs";
        for (auto it = o.begin(); it != o.end(); ++it) {
          if (it->first == "x")
            x = static_cast<double>(it->second);
          else if (it->first == "y")
            y = static_cast<double>(it->second);
          else if (it->first == "yaw")
            yaw = static_cast<double>(it->second);
          else if (it->first == "size_x")
            size_x = static_cast<double>(it->second);
          else if (it->first == "size_y")
            size_y = static_cast<double>(it->second);
          else if (it->first == "name")
            name = static_cast<std::string>(it->second);
          else if (it->first == "type")
            type = static_cast<std::string>(it->second);
        }
        ROS_INFO_STREAM("  Obstacle " << i << ": {name: " << name << ", type: " << type << ", x: " << x << ", y: " << y
                                      << ", yaw: " << yaw << ", size_x: " << size_x << ", size_y: " << size_y << "}");
        ellis_planner::AddObstacle add_obs;
        add_obs.request.type = type;
        add_obs.request.name = name;
        add_obs.request.pose.position.x = x;
        add_obs.request.pose.position.y = y;
        tf2::Quaternion orien;
        orien.setRPY(0, 0, yaw);
        add_obs.request.pose.orientation.x = orien.x();
        add_obs.request.pose.orientation.y = orien.y();
        add_obs.request.pose.orientation.z = orien.z();
        add_obs.request.pose.orientation.w = orien.w();
        add_obs.request.size.x = size_x;
        add_obs.request.size.y = size_y;
        if (add_obstacle_client_.call(add_obs))
          ROS_INFO("[ExperimentManager] Adding obstacle...");
        else
          ROS_WARN("[ExperimentManager] Failed to call service to add obstacle!");
      }
    } else {
      ROS_WARN("[ExperimentManager] No obstacles specified");
    }

    control_history_pub_ = nh_.advertise<ellis_planner::ControlHistory>("/exp/control_history", 10);

    return true;
  }

  void Run() {
    waypoint_index_ = 0;

    // Clear any existing execution errors cached from previous experiments.
    std_srvs::Trigger srv;
    if (clear_execution_errors_client_.call(srv))
      ROS_INFO("[ExperimentManager] Clearing old execution errors...");
    else
      ROS_WARN("[ExperimentManager] Failed to call service to clear old execution errors!");

    ros::Rate rate(5.0);
    while (ros::ok()) {
      switch (state_) {
        case READY: {
          if (waypoint_index_ < waypoint_.size()) {
            ROS_INFO_STREAM("Ready to execute waypoint " << waypoint_index_ + 1 << " of " << waypoint_.size()
                                                         << ", press ENTER to continue...");
            std::cin.get();

            const auto& next_waypoint = waypoint_[waypoint_index_];
            const auto& next_planner_type = planner_type_[waypoint_index_];

            // Change to the specified planner.
            cfg_.Set<std::string>("planner", next_planner_type);
            if (!cfg_.Reconfigure()) {
              ROS_ERROR_STREAM("[ExperimentManager] Could change planner type to \"" << next_planner_type << "\"!");
              state_ = ERROR;
              break;
            }

            if (!MoveTo(next_waypoint[0], next_waypoint[1], next_waypoint[2])) {
              ROS_ERROR_STREAM("Move to (" << next_waypoint[0] << ", " << next_waypoint[1] << ", " << next_waypoint[2]
                                           << ") failed!");
            } else {
              waypoint_index_++;
              state_ = MOVING;
            }
          }
          break;
        }
        case MOVING: {
          ROS_DEBUG_STREAM("Moving...");
          break;
        }
        case REPLAN_NEEDED: {
          // Change to the specified planner.
          cfg_.Set<std::string>("planner", planner_type_[waypoint_index_ - 1]);
          if (!cfg_.Reconfigure()) {
            ROS_ERROR_STREAM("[ExperimentManager] Could change planner type to \"" << planner_type_[waypoint_index_ - 1]
                                                                                   << "\"!");
            state_ = ERROR;
            break;
          }

          const auto& waypoint = waypoint_[waypoint_index_ - 1];
          ROS_INFO_STREAM("Replanning to waypoint " << waypoint_index_ - 1 << ": (" << waypoint[0] << ", "
                                                    << waypoint[1] << ", " << waypoint[2] << ")");
          if (!MoveTo(waypoint[0], waypoint[1], waypoint[2])) {
            ROS_ERROR_STREAM("Move to (" << waypoint[0] << ", " << waypoint[1] << ", " << waypoint[2] << ") failed!");
          } else {
            state_ = MOVING;
          }
          break;
        }
        case ERROR: {
          ROS_DEBUG_STREAM("[ExperimentManager] Error...");
          break;
        }
        default:
          break;
      }

      ros::spinOnce();
      rate.sleep();
    }

    ros::spin();
  }

 private:
  bool ZeroYaw() {
    ROS_INFO("[ExperimentManager] Zeroing yaw...");
    double curr_x, curr_y, curr_z, curr_yaw;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_ERROR("[ExperimentManager] Failed to get robot's current pose!");
      return false;
    }

    if (std::abs(curr_yaw) < 1e-3) {
      ROS_WARN_STREAM("[ExperimentManager] Robot's current yaw (" << curr_yaw << ") is close enough to zero!");
      return true;
    }

    cfg_.Set<std::string>("planner", "trapezoidal");
    if (!cfg_.Reconfigure()) {
      ROS_ERROR("[ExperimentManager] Could not change to trapezoidal planner!");
      return false;
    }

    return MoveTo(curr_x, curr_y, 0.0);
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
    ROS_INFO_STREAM("[ExperimentManager] Sending motion goal with position ("
                    << state.pose.position.x << ", " << state.pose.position.y << ", " << state.pose.position.z
                    << ") and yaw " << yaw << "...");
    if (!mobility_client_.SendGoal(goal)) {
      ROS_ERROR("[ExperimentManager] Mobility client did not accept goal");
      return false;
    }

    return true;
  }

  // Puts the robot into a "station keeping" mode to stop all motion.
  bool Stop() {
    ff_msgs::MotionGoal goal;
    goal.flight_mode = "nominal";
    goal.command = ff_msgs::MotionGoal::STOP;
    if (!mobility_client_.SendGoal(goal)) {
      ROS_ERROR("[ExperimentManager] Mobility client did not accept stop goal");
      return false;
    }

    return true;
  }

  bool GetPose(double& x, double& y, double& z, double& yaw) {
    geometry_msgs::TransformStamped world_to_body;
    try {
      world_to_body =
        tf_buffer_.lookupTransform(std::string(FRAME_NAME_WORLD), std::string(FRAME_NAME_BODY), ros::Time(0));
    } catch (const tf2::TransformException& ex) {
      ROS_ERROR_STREAM("[ExperimentManager] Error: " << ex.what());
      return false;
    }

    x = world_to_body.transform.translation.x;
    y = world_to_body.transform.translation.y;
    z = world_to_body.transform.translation.z;
    yaw = tf2::getYaw(world_to_body.transform.rotation);
    return true;
  }

  bool AtGoal(double goal_dist_tol = 0.15) {
    int goal_waypoint_index = waypoint_index_ - 1;
    if (goal_waypoint_index < 0 || goal_waypoint_index >= waypoint_.size()) {
      ROS_WARN_STREAM("[ExperimentManager] Goal waypoint index " << goal_waypoint_index << " out-of-bounds!");
      return false;
    }
    const auto& goal_waypoint = waypoint_[goal_waypoint_index];

    double curr_x = 0.0, curr_y = 0.0, curr_z = 0.0, curr_yaw = 0.0;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_WARN_STREAM("[ExperimentManager] Could not check if at goal, because could not get current pose!");
      return false;
    }

    double dist = std::sqrt(std::pow(goal_waypoint[0] - curr_x, 2) + std::pow(goal_waypoint[1] - curr_y, 2));
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
      ROS_ERROR("[ExperimentManager] Failed to find root of error!");
      return false;
    }

    ROS_WARN_STREAM("[ExperimentManager] Found root of error at index " << index << " (history size is "
                                                                        << control_feedback_history_.size() << ")");
    const auto& fb = control_feedback_history_[index];
    ROS_WARN_STREAM("[ExperimentManager]   pos err: "
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
      ROS_ERROR("[ExperimentManager] Failed to call service to report an execution error");
      return false;
    }

    return true;
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
        ROS_ERROR("[ExperimentManager] Timeout on connect");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
        ROS_ERROR("[ExperimentManager] Timeout on active");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
        ROS_ERROR("[ExperimentManager] Timeout on response");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
        ROS_ERROR("[ExperimentManager] Timeout on deadline");
        break;
      case ff_util::FreeFlyerActionState::Enum::SUCCESS:
      case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
      case ff_util::FreeFlyerActionState::Enum::ABORTED: {
        ROS_INFO_STREAM("[ExperimentManager] Result: " << result->fsm_result);
        switch (result->response) {
          case ff_msgs::MotionResult::ALREADY_THERE:
          case ff_msgs::MotionResult::SUCCESS:
          case ff_msgs::MotionResult::PREEMPTED: {
            if (AtGoal()) {
              // Experiment is done.
              ROS_INFO("[ExperimentManager] Experiment is done!");
              state_ = READY;
            } else {
              ROS_WARN("[ExperimentManager] Not done yet! Replanning...");
              state_ = REPLAN_NEEDED;
            }
            break;
          }
            // TODO(eratner) What is the difference between POSITION_ENDPOINT and POSITION?
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION_ENDPOINT:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_ATTITUDE:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION: {
            // A execution failure occurred!
            ROS_WARN("[ExperimentManager] Something unexpected occurred!");

            // Stop the robot.
            if (stop_robot_on_execution_error_) {
              ROS_WARN("[ExperimentManager] Stopping robot...");
              if (!Stop()) ROS_ERROR("[ExperimentManager] Failed to stop the robot!");
            }

            // TODO(eratner) experimental-- publish control history
            ROS_INFO_STREAM("[ExperimentManager] Publishing control history with "
                            << control_history_.actual_pose.size() << " poses...");
            control_history_pub_.publish(control_history_);

            if (ReportExecutionError()) {
              ROS_INFO("[ExperimentManager] Reported execution error...");
              state_ = REPLAN_NEEDED;
            } else {
              ROS_ERROR("[ExperimentManager] Failed to report an execution error!");
              state_ = ERROR;
            }

            // TODO(eratner) A little hacky
            // ZeroYaw();
            // control_feedback_history_.clear();
            // control_history_.desired_pose.clear();
            // control_history_.actual_pose.clear();


            // for (const auto& f : control_feedback_history_) {
            //   ROS_WARN_STREAM("  pos err: " << f.error_position << ", att err: " << f.error_attitude << ", x: "
            //                                 << f.setpoint.pose.position.x << ", y: " << f.setpoint.pose.position.y
            //                                 << ", yaw: " << tf2::getYaw(f.setpoint.pose.orientation) << ", vel x: "
            //                                 << f.setpoint.twist.linear.x << ", vel y: " << f.setpoint.twist.linear.y
            //                                 << ", vel yaw: " << f.setpoint.twist.angular.z);
            // }

            // // Report the execution error.
            // std_srvs::Trigger srv;
            // if (report_execution_error_client_.call(srv)) {
            //   ROS_INFO("[ExperimentManager] Reporting execution error...");
            //   state_ = REPLAN_NEEDED;
            // } else {
            //   ROS_ERROR("[ExperimentManager] Failed to call service to report an execution error");
            //   state_ = ERROR;
            // }

            break;
          }
          default:
            // TODO(eratner) What now?
            break;
        }
      }
      default:
        break;
    }
  }

  void FeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
    // std::cout << '\r' << std::flush;
    // std::cout << std::fixed << std::setprecision(2) << "POS: " << 1000.00 * feedback->progress.error_position << " mm
    // "
    //           << "ATT: " << 57.2958 * feedback->progress.error_attitude << " deg "
    //           << "VEL: " << 1000.00 * feedback->progress.error_velocity << " mm/s "
    //           << "OMEGA: " << 57.2958 * feedback->progress.error_omega << " deg/s "
    //           << "[" << feedback->state.fsm_state << "]   ";
    ROS_INFO_STREAM("  Setpoint " << feedback->progress.index << ": pos: ("
                                  << feedback->progress.setpoint.pose.position.x << ", "
                                  << feedback->progress.setpoint.pose.position.y << ", "
                                  << feedback->progress.setpoint.pose.position.z
                                  << "), yaw: " << tf2::getYaw(feedback->progress.setpoint.pose.orientation));

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

  ros::NodeHandle nh_;
  ros::NodeHandle p_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> mobility_client_;
  State state_;
  ff_util::ConfigClient cfg_;
  std::vector<std::array<double, 3>> waypoint_;
  int waypoint_index_;
  std::vector<std::string> planner_type_;
  ros::ServiceClient add_obstacle_client_;
  ros::ServiceClient clear_obstacles_client_;
  ros::ServiceClient report_execution_error_client_;
  ros::ServiceClient clear_execution_errors_client_;
  boost::circular_buffer<ff_msgs::ControlFeedback> control_feedback_history_;
  bool stop_robot_on_execution_error_;

  ellis_planner::ControlHistory control_history_;
  ros::Publisher control_history_pub_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "experiment_manager", ros::init_options::AnonymousName);

  ExperimentManager mgr;
  if (!mgr.Init()) {
    ROS_ERROR("[ExperimentManager] Failed to initialize");
    return -1;
  }

  mgr.Run();
  return 0;
}
