#include <ros/ros.h>
#include <ff_util/config_client.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <ff_msgs/MotionAction.h>
#include <discrepancy_planner/util.h>
#include <std_srvs/Trigger.h>

class Experiment {
 public:
  enum State { WAITING = 0, REPLAN_NEEDED, DONE };

  Experiment() : cfg_(&nh_, NODE_CHOREOGRAPHER) {}

  bool Init() {
    // Use the discrepancy planner for motion planning.
    ROS_INFO("[Experiment] Setting planner to type discrepancy");
    cfg_.Set<std::string>("planner", "discrepancy");
    if (!cfg_.Reconfigure()) {
      ROS_ERROR("[Experiment] Could change planner type in the choreographer");
      return false;
    }

    // Connect to the mobility action server.
    mobility_client_.SetConnectedTimeout(30.0);
    mobility_client_.SetActiveTimeout(30.0);
    mobility_client_.SetResponseTimeout(30.0);
    mobility_client_.SetFeedbackCallback(
      std::bind(&Experiment::FeedbackCallback, this, std::placeholders::_1));
    mobility_client_.SetResultCallback(std::bind(&Experiment::ResultCallback,
                                                 this, std::placeholders::_1,
                                                 std::placeholders::_2));
    mobility_client_.Create(&nh_, ACTION_MOBILITY_MOTION);
    while (ros::ok()) {
      if (mobility_client_.IsConnected()) {
        ROS_INFO("[Experiment] Mobility client connected to action server!");
        break;
      } else
        ROS_WARN("[Experiment] Mobility client waiting to connect...");

      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }

    add_discrepancy_client_ = nh_.serviceClient<std_srvs::Trigger>(
      "/mob/planner_discrepancy/add_discrepancy");

    state_ = REPLAN_NEEDED;

    return true;
  }

  void Run() {
    ros::Rate rate(5.0);
    while (state_ != DONE && ros::ok()) {
      switch (state_) {
        case WAITING:
          break;
        case REPLAN_NEEDED:
          ROS_INFO("[Experiment] Replan needed");
          MoveToGoal();
          state_ = WAITING;
          break;
        default:
          break;
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  bool MoveToGoal() {
    // Send a mobility command to plan and execute a path to the goal.
    ff_msgs::MotionGoal goal;
    goal.flight_mode = "nominal";
    goal.command = ff_msgs::MotionGoal::MOVE;
    geometry_msgs::PoseStamped state;
    state.header.stamp = ros::Time::now();
    state.pose.position.x = nh_.param<double>("discrepancy_planner/goal/x", 0);
    state.pose.position.y = nh_.param<double>("discrepancy_planner/goal/y", 0);
    state.pose.position.z =
      nh_.param<double>("discrepancy_planner/goal/z", -0.675);
    double goal_yaw = nh_.param<double>("discrepancy_planner/goal/yaw", 0);
    auto orien = discrepancy_planner::RPYToQuaternion(0, 0, goal_yaw);
    state.pose.orientation.x = orien.x();
    state.pose.orientation.y = orien.y();
    state.pose.orientation.z = orien.z();
    state.pose.orientation.w = orien.w();
    goal.states = {state};
    ROS_INFO_STREAM("[Experiment] Sending motion goal with position ("
                    << state.pose.position.x << ", " << state.pose.position.y
                    << ", " << state.pose.position.z << ") and yaw " << goal_yaw
                    << "...");
    if (!mobility_client_.SendGoal(goal)) {
      ROS_ERROR("[Experiment] Mobility client did not accept goal");
      return false;
    }

    return true;
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
                      ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
        ROS_ERROR("[Experiment] Timeout on connect");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
        ROS_ERROR("[Experiment] Timeout on active");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
        ROS_ERROR("[Experiment] Timeout on response");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
        ROS_ERROR("[Experiment] Timeout on deadline");
        break;
      case ff_util::FreeFlyerActionState::Enum::SUCCESS:
      case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
      case ff_util::FreeFlyerActionState::Enum::ABORTED: {
        ROS_INFO_STREAM("[Experiment] Result: " << result->fsm_result);
        switch (result->response) {
          case ff_msgs::MotionResult::ALREADY_THERE:
          case ff_msgs::MotionResult::SUCCESS:
          case ff_msgs::MotionResult::PREEMPTED: {
            // Experiment is done.
            state_ = DONE;
            break;
          }
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION: {
            // A discrepancy occurred!
            ROS_WARN("[Experiment] Discrepancy!");

            std_srvs::Trigger srv;
            if (add_discrepancy_client_.call(srv)) {
              ROS_INFO("[Experiment] Added discrepancy!");
            } else {
              ROS_ERROR(
                "[Experiment] Failed to call service to add discrepancy");
              // TODO ...
            }

            state_ = REPLAN_NEEDED;
            break;
          }
          default:
            state_ = REPLAN_NEEDED;
            break;
        }
      }
      default:
        break;
    }
  }

  void FeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
    std::cout << '\r' << std::flush;
    std::cout << std::fixed << std::setprecision(2)
              << "POS: " << 1000.00 * feedback->progress.error_position
              << " mm "
              << "ATT: " << 57.2958 * feedback->progress.error_attitude
              << " deg "
              << "VEL: " << 1000.00 * feedback->progress.error_velocity
              << " mm/s "
              << "OMEGA: " << 57.2958 * feedback->progress.error_omega
              << " deg/s "
              << "[" << feedback->state.fsm_state << "]   ";
  }

  ros::NodeHandle nh_;
  ff_util::ConfigClient cfg_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> mobility_client_;
  ros::ServiceClient add_discrepancy_client_;
  State state_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "run_experiment", ros::init_options::AnonymousName);

  Experiment experiment;
  if (!experiment.Init()) {
    ROS_ERROR("[Experiment] Failed to initialize");
    return -1;
  }

  experiment.Run();
  return 0;
}
