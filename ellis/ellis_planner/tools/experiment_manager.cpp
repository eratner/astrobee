// Copyright 2022 Ellis Ratner
#include <ros/ros.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <ff_msgs/MotionAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <array>
#include <string>
#include <vector>

class ExperimentManager {
 public:
  ExperimentManager() : tf_listener_(tf_buffer_), state_(READY) {}

  ~ExperimentManager() {}

  enum State { READY = 0, MOVING, ERROR };

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

    return true;
  }

  void Run() {
    std::vector<std::array<double, 3>> waypoints;  // TODO(eratner) Read these from a config
    waypoints = {{0, 0, 0}, {-0.45, 0.35, 0}, {0.7, 0.35, 0}};
    int waypoint_index = 0;

    // TODO(eratner) set face forward off

    ros::Rate rate(5.0);
    while (ros::ok()) {
      switch (state_) {
        case READY:
          if (waypoint_index < waypoints.size()) {
            const auto& next_waypoint = waypoints[waypoint_index];
            if (!MoveTo(next_waypoint[0], next_waypoint[1], next_waypoint[2])) {
              ROS_ERROR_STREAM("Move to (" << next_waypoint[0] << ", " << next_waypoint[1] << ", " << next_waypoint[2]
                                           << ") failed!");
            } else {
              waypoint_index++;
              state_ = MOVING;
            }
          }
          break;
        case MOVING:
        case ERROR:
        default:
          break;
      }

      ros::spinOnce();
      rate.sleep();
    }

    ros::spin();
  }

 private:
  bool MoveTo(double x, double y, double yaw = 0.0) {
    // Get the robot's current pose.
    double curr_x, curr_y, curr_z, curr_yaw;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_ERROR("[ExperimentManager] Failed to get robot's current pose!");
      return false;
    }

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

  bool GetPose(double& x, double& y, double& z, double& yaw) {
    geometry_msgs::TransformStamped world_to_body;
    try {
      world_to_body =
        tf_buffer_.lookupTransform(std::string(FRAME_NAME_WORLD), std::string(FRAME_NAME_BODY), ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_STREAM("[ExperimentManager] Error: " << ex.what());
      return false;
    }

    x = world_to_body.transform.translation.x;
    y = world_to_body.transform.translation.y;
    z = world_to_body.transform.translation.z;
    yaw = tf2::getYaw(world_to_body.transform.rotation);
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
            // Experiment is done.
            state_ = READY;
            break;
          }
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_ATTITUDE:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION: {
            // A discrepancy occurred!
            ROS_WARN("[ExperimentManager] Something unexpected occurred!");
            state_ = ERROR;

            // TODO(eratner) Do something!
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
    std::cout << '\r' << std::flush;
    std::cout << std::fixed << std::setprecision(2) << "POS: " << 1000.00 * feedback->progress.error_position << " mm "
              << "ATT: " << 57.2958 * feedback->progress.error_attitude << " deg "
              << "VEL: " << 1000.00 * feedback->progress.error_velocity << " mm/s "
              << "OMEGA: " << 57.2958 * feedback->progress.error_omega << " deg/s "
              << "[" << feedback->state.fsm_state << "]   ";
  }

  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> mobility_client_;
  State state_;
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
