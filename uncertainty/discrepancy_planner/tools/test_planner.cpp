#include <ros/ros.h>
#include <ff_util/config_client.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <ff_msgs/MotionAction.h>
#include <discrepancy_planner/util.h>

bool sent = false;

void ResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::MotionResultConstPtr const& result) {
  switch (result_code) {
    // Result will be a null pointer
    case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
      std::cout << "Timeout on connecting to action" << std::endl;
      break;
    case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
      std::cout << "Timeout on action going active" << std::endl;
      break;
    case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
      std::cout << "Timeout on receiving a response" << std::endl;
      break;
    case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
      std::cout << "Timeout on result deadline" << std::endl;
      break;
    // Result expected
    case ff_util::FreeFlyerActionState::Enum::SUCCESS:
    case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
    case ff_util::FreeFlyerActionState::Enum::ABORTED: {
      std::cout << std::endl
                << "Result: " << result->fsm_result << " (response: " << result->response << ")" << std::endl;
    }
    default:
      break;
  }
  ros::shutdown();
}

void FeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
  std::cout << '\r' << std::flush;
  std::cout << std::fixed << std::setprecision(2) << "POS: " << 1000.00 * feedback->progress.error_position << " mm "
            << "ATT: " << 57.2958 * feedback->progress.error_attitude << " deg "
            << "VEL: " << 1000.00 * feedback->progress.error_velocity << " mm/s "
            << "OMEGA: " << 57.2958 * feedback->progress.error_omega << " deg/s "
            << "[" << feedback->state.fsm_state << "]   ";
}

void ConnectedCallback(ff_util::FreeFlyerActionClient<ff_msgs::MotionAction>* mobility_client, ros::NodeHandle* nh) {
  // Wait for the client to connect.
  if (!mobility_client->IsConnected()) return;

  // Make sure we send the command only once.
  if (sent)
    return;
  else
    sent = true;

  // Send the mobility command.
  ff_msgs::MotionGoal goal;
  goal.flight_mode = "nominal";
  goal.command = ff_msgs::MotionGoal::MOVE;
  geometry_msgs::PoseStamped state;
  state.header.stamp = ros::Time::now();
  state.pose.position.x = nh->param<double>("discrepancy_planner/experiment/goal/x", 0);
  state.pose.position.y = nh->param<double>("discrepancy_planner/experiment/goal/y", 0);
  state.pose.position.z = nh->param<double>("discrepancy_planner/experiment/goal/z", -0.675);
  auto orien =
    discrepancy_planner::RPYToQuaternion(0, 0, nh->param<double>("discrepancy_planner/experiment/goal/yaw", 0));
  state.pose.orientation.x = orien.x();
  state.pose.orientation.y = orien.y();
  state.pose.orientation.z = orien.z();
  state.pose.orientation.w = orien.w();
  goal.states = {state};
  ROS_INFO_STREAM("Sending motion goal with position (" << state.pose.position.x << ", " << state.pose.position.y
                                                        << ", " << state.pose.position.z << ")...");
  if (!mobility_client->SendGoal(goal)) ROS_ERROR("Mobility client did not accept goal");
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_planner", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ff_util::ConfigClient cfg(&nh, NODE_CHOREOGRAPHER);
  ROS_INFO("Setting planner to type discrepancy");
  cfg.Set<std::string>("planner", "discrepancy");
  if (!cfg.Reconfigure()) {
    ROS_ERROR("Could reconfigure the planner type in the choreographer");
    return -1;
  }

  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> mobility_client;

  mobility_client.SetConnectedTimeout(30.0);
  mobility_client.SetActiveTimeout(30.0);
  mobility_client.SetResponseTimeout(30.0);
  mobility_client.SetFeedbackCallback(std::bind(FeedbackCallback, std::placeholders::_1));
  mobility_client.SetResultCallback(std::bind(ResultCallback, std::placeholders::_1, std::placeholders::_2));
  mobility_client.SetConnectedCallback(std::bind(ConnectedCallback, &mobility_client, &nh));
  mobility_client.Create(&nh, ACTION_MOBILITY_MOTION);

  ros::spin();
  return 0;
}
