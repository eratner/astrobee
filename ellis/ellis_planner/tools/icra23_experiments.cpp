// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ros/ros.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <ff_util/config_client.h>
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/SetZones.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_msgs/TFMessage.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <boost/circular_buffer.hpp>
#include <ellis_planner/ReportExecutionError.h>
#include <ellis_planner/AddObstacle.h>
#include <ellis_planner/AddDisturbance.h>
#include <ellis_planner/ControlHistory.h>
#include <ellis_planner/PlanningInfo.h>
#include <ellis_planner/clock.h>
#include <ellis_planner/environment.h>
#include <ellis_planner/search.h>
#include <ellis_planner/rectangle_collision_object.h>
#include <yaml-cpp/yaml.h>
#include <XmlRpcException.h>
#include <array>
#include <algorithm>
#include <string>
#include <vector>
#include <random>
#include <fstream>

class Experiments {
 public:
  enum Method { OURS = 0, REPLANNING, CMAX, NUM_METHODS };

  enum State { REPLAN_NEEDED = 0, EXECUTING, DONE, ERROR };

  struct ObstacleInfo {
    explicit ObstacleInfo(const std::string& type = "rectangle", const std::string& name = "obs", double x = 0.0,
                          double y = 0.0, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, double size_x = 0.0,
                          double size_y = 0.0, bool known = true, bool moveable = false, double mass = 1.0,
                          double restitution_coeff = 0.0, double friction_coeff = 1.0, bool randomize = false,
                          double random_x_lower = 0.0, double random_x_upper = 0.0, double random_y_lower = 0.0,
                          double random_y_upper = 0.0, double random_roll_lower = 0.0, double random_roll_upper = 0.0,
                          double random_pitch_lower = 0.0, double random_pitch_upper = 0.0,
                          double random_yaw_lower = 0.0, double random_yaw_upper = 0.0,
                          double random_size_x_lower = 0.0, double random_size_x_upper = 0.0,
                          double random_size_y_lower = 0.0, double random_size_y_upper = 0.0,
                          double random_mass_lower = 0.0, double random_mass_upper = 0.0,
                          double random_restitution_lower = 0.0, double random_restitution_upper = 0.0,
                          double random_friction_lower = 0.0, double random_friction_upper = 0.0)
        : type_(type),
          name_(name),
          x_(x),
          y_(y),
          roll_(roll),
          pitch_(pitch),
          yaw_(yaw),
          size_x_(size_x),
          size_y_(size_y),
          known_(known),
          moveable_(moveable),
          mass_(mass),
          restitution_coeff_(restitution_coeff),
          friction_coeff_(friction_coeff),
          randomize_(randomize),
          dist_x_(random_x_lower, random_x_upper),
          dist_y_(random_y_lower, random_y_upper),
          dist_roll_(random_roll_lower, random_roll_upper),
          dist_pitch_(random_pitch_lower, random_pitch_upper),
          dist_yaw_(random_yaw_lower, random_yaw_upper),
          dist_size_x_(random_size_x_lower, random_size_x_upper),
          dist_size_y_(random_size_y_lower, random_size_y_upper),
          dist_mass_(random_mass_lower, random_mass_upper),
          dist_restitution_coeff_(random_restitution_lower, random_restitution_upper),
          dist_friction_coeff_(random_friction_lower, random_friction_upper) {}

    void Randomize(std::mt19937& gen,
                   const std::vector<ObstacleInfo>& do_not_collide_with = std::vector<ObstacleInfo>(),
                   unsigned int max_tries = 100) {
      int i = 0;
      double x = x_, y = y_, roll = roll_, pitch = pitch_, yaw = yaw_, size_x = size_x_, size_y = size_y_;
      while (i < max_tries) {
        x = x_ + dist_x_(gen);
        y = y_ + dist_y_(gen);
        roll = roll_ + dist_roll_(gen);
        pitch = pitch_ + dist_pitch_(gen);
        yaw = yaw_ + dist_yaw_(gen);
        size_x = size_x_ + dist_size_x_(gen);
        size_y = size_y_ + dist_size_y_(gen);

        ellis_planner::RectangleCollisionObject rect("obj", x, y, yaw, size_x, size_y);
        bool no_collisions = true;
        for (const auto& o : do_not_collide_with) {
          ellis_planner::RectangleCollisionObject other_rect("obj", o.x_, o.y_, o.yaw_, o.size_x_, o.size_y_);
          if (rect.InCollision(&other_rect)) {
            no_collisions = false;
            break;
          }
        }

        if (no_collisions) break;

        i++;
      }
      x_ = x;
      y_ = y;
      roll_ = roll;
      pitch_ = pitch;
      yaw_ = yaw;
      size_x_ = size_x;
      size_y_ = size_y;

      mass_ = std::max(1e-3, mass_ + dist_mass_(gen));
      restitution_coeff_ = std::min(std::max(0.0, restitution_coeff_ + dist_restitution_coeff_(gen)), 1.0);
      friction_coeff_ = std::max(0.0, friction_coeff_ + dist_friction_coeff_(gen));
    }

    YAML::Node ToYaml() const {
      YAML::Node node;
      node["type"] = type_;
      node["name"] = name_;
      node["x"] = x_;
      node["y"] = y_;
      node["roll"] = roll_;
      node["pitch"] = pitch_;
      node["yaw"] = yaw_;
      node["size_x"] = size_x_;
      node["size_y"] = size_y_;
      node["moveable"] = moveable_;
      node["mass"] = mass_;
      node["restitution_coeff"] = restitution_coeff_;
      node["friction_coeff"] = friction_coeff_;
      return node;
    }

    std::string type_;
    std::string name_;
    double x_;
    double y_;
    double roll_;
    double pitch_;
    double yaw_;
    double size_x_;
    double size_y_;
    bool known_;
    bool moveable_;
    double mass_;
    double restitution_coeff_;
    double friction_coeff_;
    bool randomize_;

    std::uniform_real_distribution<> dist_x_;
    std::uniform_real_distribution<> dist_y_;
    std::uniform_real_distribution<> dist_roll_;
    std::uniform_real_distribution<> dist_pitch_;
    std::uniform_real_distribution<> dist_yaw_;
    std::uniform_real_distribution<> dist_size_x_;
    std::uniform_real_distribution<> dist_size_y_;
    std::uniform_real_distribution<> dist_mass_;
    std::uniform_real_distribution<> dist_restitution_coeff_;
    std::uniform_real_distribution<> dist_friction_coeff_;
  };

  struct DisturbanceInfo {
    explicit DisturbanceInfo(double center_x = 0.0, double center_y = 0.0, double size_x = 0.0, double size_y = 0.0,
                             double force_x = 0.0, double force_y = 0.0, bool randomize = false,
                             double random_center_x_lower = 0.0, double random_center_x_upper = 0.0,
                             double random_center_y_lower = 0.0, double random_center_y_upper = 0.0,
                             double random_size_x_lower = 0.0, double random_size_x_upper = 0.0,
                             double random_size_y_lower = 0.0, double random_size_y_upper = 0.0,
                             double random_force_angle_lower = 0.0, double random_force_angle_upper = 0.0,
                             double random_force_mag_lower = 0.0, double random_force_mag_upper = 0.0)
        : center_x_(center_x),
          center_y_(center_y),
          size_x_(size_x),
          size_y_(size_y),
          force_x_(force_x),
          force_y_(force_y),
          randomize_(randomize),
          dist_center_x_(random_center_x_lower, random_center_x_upper),
          dist_center_y_(random_center_y_lower, random_center_y_upper),
          dist_size_x_(random_size_x_lower, random_size_x_upper),
          dist_size_y_(random_size_y_lower, random_size_y_upper),
          dist_force_angle_(random_force_angle_lower, random_force_angle_upper),
          dist_force_mag_(random_force_mag_lower, random_force_mag_upper) {}

    void Randomize(std::mt19937& gen) {
      center_x_ += dist_center_x_(gen);
      center_y_ += dist_center_y_(gen);
      size_x_ += dist_size_x_(gen);
      size_y_ += dist_size_y_(gen);

      double force_mag = std::sqrt(std::pow(force_x_, 2) + std::pow(force_y_, 2));
      double force_angle = std::atan2(force_y_ / force_mag, force_x_ / force_mag);

      force_angle += dist_force_angle_(gen);
      force_mag += dist_force_mag_(gen);

      force_x_ = force_mag * std::cos(force_angle);
      force_y_ = force_mag * std::sin(force_angle);
    }

    YAML::Node ToYaml() const {
      YAML::Node node;
      node["center_x"] = center_x_;
      node["center_y"] = center_y_;
      node["size_x"] = size_x_;
      node["size_y"] = size_y_;
      node["force_x"] = force_x_;
      node["force_y"] = force_y_;
      return node;
    }

    double center_x_;
    double center_y_;
    double size_x_;
    double size_y_;
    double force_x_;
    double force_y_;
    bool randomize_;

    std::uniform_real_distribution<> dist_center_x_;
    std::uniform_real_distribution<> dist_center_y_;
    std::uniform_real_distribution<> dist_size_x_;
    std::uniform_real_distribution<> dist_size_y_;
    std::uniform_real_distribution<> dist_force_angle_;
    std::uniform_real_distribution<> dist_force_mag_;
  };

  struct Output {
    Method method_;
    int random_seed_;
    std::string path_to_bag_;
    double start_x_;
    double start_y_;
    double start_yaw_;
    double goal_x_;
    double goal_y_;
    double goal_yaw_;
    double execution_time_sec_;
    double total_distance_;
    bool reached_goal_;
    std::vector<ObstacleInfo> obstacles_;
    std::vector<DisturbanceInfo> disturbances_;

    YAML::Node ToYaml() const {
      YAML::Node node;
      node["method"] = ToString(method_);
      node["random_seed"] = random_seed_;
      node["path_to_bag"] = path_to_bag_;
      node["start_x"] = start_x_;
      node["start_y"] = start_y_;
      node["start_yaw"] = start_yaw_;
      node["goal_x"] = goal_x_;
      node["goal_y"] = goal_y_;
      node["goal_yaw"] = goal_yaw_;
      node["execution_time_sec"] = execution_time_sec_;
      node["total_distance"] = total_distance_;
      node["reached_goal"] = reached_goal_;

      YAML::Node obstacles_node;
      for (const auto& o : obstacles_) obstacles_node.push_back(o.ToYaml());
      node["obstacles"] = obstacles_node;

      YAML::Node disturbances_node;
      for (const auto& d : disturbances_) disturbances_node.push_back(d.ToYaml());
      node["disturbances"] = disturbances_node;

      return node;
    }
  };

  Experiments()
      : p_nh_("~"), tf_listener_(tf_buffer_), cfg_(&nh_, NODE_CHOREOGRAPHER), control_feedback_history_(1000) {}

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

    set_zones_client_ = nh_.serviceClient<ff_msgs::SetZones>("/mob/set_zones");
    add_simulator_obstacle_client_ = nh_.serviceClient<ellis_planner::AddObstacle>("/obstacle_manager/add_obstacle");
    clear_simulator_obstacles_client_ = nh_.serviceClient<std_srvs::Trigger>("/obstacle_manager/clear_obstacles");
    add_planner_obstacle_client_ = nh_.serviceClient<ellis_planner::AddObstacle>("/mob/ellis_planner/add_obstacle");
    clear_planner_obstacles_client_ = nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/clear_obstacles");
    report_execution_error_client_ =
      nh_.serviceClient<ellis_planner::ReportExecutionError>("/mob/ellis_planner/report_execution_error");
    clear_execution_errors_client_ = nh_.serviceClient<std_srvs::Trigger>("/mob/ellis_planner/clear_execution_errors");
    add_disturbance_client_ = nh_.serviceClient<ellis_planner::AddDisturbance>("/disturbance_manager/add_disturbance");
    clear_disturbances_client_ = nh_.serviceClient<std_srvs::Trigger>("/disturbance_manager/clear_disturbances");

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
      if (add_simulator_obstacle_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for add simulator obstacle server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (add_planner_obstacle_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for add planner obstacle server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (clear_planner_obstacles_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for clear planner obstacles server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (add_disturbance_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for add disturbance server...");
        ros::spinOnce();
      }
    }

    while (ros::ok()) {
      if (clear_disturbances_client_.waitForExistence(ros::Duration(0.25))) {
        break;
      } else {
        ROS_WARN("[Experiments] Waiting for clear disturbances server...");
        ros::spinOnce();
      }
    }

    control_history_pub_ = nh_.advertise<ellis_planner::ControlHistory>("/exp/control_history", 10);

    // TODO(eratner) Read topic name from config
    std::string vis_topic = "/mob/ellis_planner/vis";
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>(vis_topic, 1000);

    subs_ = {
      nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1,
                                         [this](const tf2_msgs::TFMessageConstPtr& msg) {
                                           if (bag_.isOpen()) bag_.write("/tf", ros::Time::now(), *msg);
                                         }),
      nh_.subscribe<tf2_msgs::TFMessage>(
        "/tf_static", 1, [this](const tf2_msgs::TFMessageConstPtr& msg) { static_tf_msgs_.push_back(*msg); }),
      nh_.subscribe<visualization_msgs::Marker>("/mob/ellis_planner/vis", 1,
                                                [this](const visualization_msgs::MarkerConstPtr& msg) {
                                                  if (bag_.isOpen())
                                                    bag_.write("/mob/ellis_planner/vis", ros::Time::now(), *msg);
                                                }),
      nh_.subscribe<ellis_planner::ControlHistory>("/exp/control_history", 1,
                                                   [this](const ellis_planner::ControlHistoryConstPtr& msg) {
                                                     if (bag_.isOpen())
                                                       bag_.write("/exp/control_history", ros::Time::now(), *msg);
                                                   }),
      nh_.subscribe<nav_msgs::Path>("/mob/choreographer/segment", 1,
                                    [this](const nav_msgs::PathConstPtr& msg) {
                                      if (bag_.isOpen())
                                        bag_.write("/mob/choreographer/segment", ros::Time::now(), *msg);
                                    }),
      nh_.subscribe<ellis_planner::PlanningInfo>("/mob/ellis_planner/info", 1,
                                                 [this](const ellis_planner::PlanningInfoConstPtr& msg) {
                                                   if (bag_.isOpen())
                                                     bag_.write("/mob/ellis_planner/info", ros::Time::now(), *msg);
                                                 }),
      nh_.subscribe<ff_msgs::MotionActionFeedback>("/mob/motion/feedback", 1,
                                                   [this](const ff_msgs::MotionActionFeedbackConstPtr& msg) {
                                                     if (bag_.isOpen())
                                                       bag_.write("/mob/motion/feedback", ros::Time::now(), *msg);
                                                   }),
      nh_.subscribe<std_msgs::String>("/mob/ellis_planner/prof", 1, [this](const std_msgs::StringConstPtr& msg) {
        planner_prof_.push_back(msg->data);
      })};

    robot_collision_size_x_ = p_nh_.param<double>("robot_collision_box/size/x", 0.30);
    robot_collision_size_y_ = p_nh_.param<double>("robot_collision_box/size/y", 0.30);

    return true;
  }

  bool Run() {
    ROS_INFO("[Run] Preparing to run experiments...");
    // Load the experiment parameters.
    int num_experiments = p_nh_.param<int>("num_experiments", 0);
    int random_seed = p_nh_.param<int>("random_seed", 0);
    std::string output_dir = p_nh_.param<std::string>("output_dir", "/tmp");
    double time_limit_sec = p_nh_.param<double>("time_limit_sec", 1000.0);
    double loop_rate_hz = p_nh_.param<double>("loop_rate_hz", 5.0);
    int start_at_index = p_nh_.param<int>("start_at_index", 0);
    ROS_INFO_STREAM("[Run]  # experiments: " << num_experiments);
    ROS_INFO_STREAM("[Run]  random seed: " << random_seed);
    ROS_INFO_STREAM("[Run]  output dir: " << output_dir);
    ROS_INFO_STREAM("[Run]  time limit (sec): " << time_limit_sec);

    // Set bounds on the map.
    if (p_nh_.hasParam("map_bounds")) {
      ff_msgs::SetZones zones;
      ff_msgs::Zone zone_msg;
      zone_msg.type = ff_msgs::Zone::KEEPIN;
      zone_msg.min.x = p_nh_.param<double>("map_bounds/min/x", -10.0);
      zone_msg.min.y = p_nh_.param<double>("map_bounds/min/y", -10.0);
      zone_msg.min.z = p_nh_.param<double>("map_bounds/min/z", -10.0);
      zone_msg.max.x = p_nh_.param<double>("map_bounds/max/x", 10.0);
      zone_msg.max.y = p_nh_.param<double>("map_bounds/max/y", 10.0);
      zone_msg.max.z = p_nh_.param<double>("map_bounds/max/z", 10.0);
      ROS_INFO_STREAM("[Run]  Setting map bounds to min: (" << zone_msg.min.x << ", " << zone_msg.min.y << ", "
                                                            << zone_msg.min.z << "), max: (" << zone_msg.max.x << ", "
                                                            << zone_msg.max.y << ", " << zone_msg.max.z << ")");
      zones.request.zones.push_back(zone_msg);
      if (!set_zones_client_.call(zones)) {
        ROS_WARN("[Run] Failed to set map bounds!");
      }
    }

    std::mt19937 gen(random_seed);

    double start_x = p_nh_.param<double>("start/pose/x", 0);
    double start_y = p_nh_.param<double>("start/pose/y", 0);
    double start_yaw = p_nh_.param<double>("start/pose/yaw", 0);
    bool randomize_start = p_nh_.param<bool>("start/randomize", false);
    ROS_INFO_STREAM("[Run]  start: (" << start_x << ", " << start_y << ", " << start_yaw << "), randomize? "
                                      << (randomize_start ? "YES" : "NO"));

    double goal_x = p_nh_.param<double>("goal/pose/x", 0);
    double goal_y = p_nh_.param<double>("goal/pose/y", 0);
    double goal_yaw = p_nh_.param<double>("goal/pose/yaw", 0);
    bool randomize_goal = p_nh_.param<bool>("goal/randomize", false);
    ROS_INFO_STREAM("[Run]  goal: (" << goal_x << ", " << goal_y << ", " << goal_yaw << "), randomize? "
                                     << (randomize_goal ? "YES" : "NO"));

    // Load obstacle info.
    std::vector<ObstacleInfo> obstacle_info;
    XmlRpc::XmlRpcValue obstacles;
    if (p_nh_.getParam("obstacles", obstacles)) {
      for (int i = 0; i < obstacles.size(); ++i) {
        XmlRpc::XmlRpcValue o = obstacles[i];
        double x = 0.0, y = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;
        double size_x = 0.0, size_y = 0.0;
        std::string type = "rectangle";
        std::string name = "obs";
        bool known = true;
        bool moveable = false;
        double mass = 1.0;
        double restitution_coeff = 0.0;
        double friction_coeff = 1.0;
        bool randomize = false;
        double random_x_lower = 0.0, random_x_upper = 0.0;
        double random_y_lower = 0.0, random_y_upper = 0.0;
        double random_roll_lower = 0.0, random_roll_upper = 0.0;
        double random_pitch_lower = 0.0, random_pitch_upper = 0.0;
        double random_yaw_lower = 0.0, random_yaw_upper = 0.0;
        double random_size_x_lower = 0.0, random_size_x_upper = 0.0;
        double random_size_y_lower = 0.0, random_size_y_upper = 0.0;
        double random_mass_lower = 0.0, random_mass_upper = 0.0;
        double random_restitution_lower = 0.0, random_restitution_upper = 0.0;
        double random_friction_lower = 0.0, random_friction_upper = 0.0;
        for (auto it = o.begin(); it != o.end(); ++it) {
          try {
            if (it->first == "x") {
              x = static_cast<double>(it->second);
            } else if (it->first == "y") {
              y = static_cast<double>(it->second);
            } else if (it->first == "roll") {
              roll = static_cast<double>(it->second);
            } else if (it->first == "pitch") {
              pitch = static_cast<double>(it->second);
            } else if (it->first == "yaw") {
              yaw = static_cast<double>(it->second);
            } else if (it->first == "size_x") {
              size_x = static_cast<double>(it->second);
            } else if (it->first == "size_y") {
              size_y = static_cast<double>(it->second);
            } else if (it->first == "name") {
              name = static_cast<std::string>(it->second);
            } else if (it->first == "type") {
              type = static_cast<std::string>(it->second);
            } else if (it->first == "known") {
              known = static_cast<bool>(it->second);
            } else if (it->first == "moveable") {
              moveable = static_cast<bool>(it->second);
            } else if (it->first == "mass") {
              mass = static_cast<double>(it->second);
            } else if (it->first == "restitution_coeff") {
              restitution_coeff = static_cast<double>(it->second);
            } else if (it->first == "friction_coeff") {
              friction_coeff = static_cast<double>(it->second);
            } else if (it->first == "randomize") {
              randomize = static_cast<bool>(it->second);
            } else if (it->first == "randomization_x") {
              random_x_lower = static_cast<double>(it->second[0]);
              random_x_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_y") {
              random_y_lower = static_cast<double>(it->second[0]);
              random_y_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_roll") {
              random_roll_lower = static_cast<double>(it->second[0]);
              random_roll_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_pitch") {
              random_pitch_lower = static_cast<double>(it->second[0]);
              random_pitch_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_yaw") {
              random_yaw_lower = static_cast<double>(it->second[0]);
              random_yaw_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_size_x") {
              random_size_x_lower = static_cast<double>(it->second[0]);
              random_size_x_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_size_y") {
              random_size_y_lower = static_cast<double>(it->second[0]);
              random_size_y_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_mass") {
              random_mass_lower = static_cast<double>(it->second[0]);
              random_mass_upper = static_cast<double>(it->second[1]);
            } else if (it->first == "randomization_friction") {
              random_friction_lower = static_cast<double>(it->second[0]);
              random_friction_upper = static_cast<double>(it->second[1]);
            }
          } catch (const XmlRpc::XmlRpcException& e) {
            ROS_ERROR_STREAM("[Run] Error: " << e.getMessage() << " (" << it->first << ")");
          }
        }
        ROS_INFO_STREAM("[Run]    Obstacle " << i << ": {name: " << name << ", type: " << type << ", x: " << x
                                             << ", y: " << y << ", yaw: " << yaw << ", size_x: " << size_x
                                             << ", size_y: " << size_y << ", known: " << (known ? "True" : "False")
                                             << ", randomize: " << (randomize ? "True" : "False") << "}");
        obstacle_info.emplace_back(type, name, x, y, roll, pitch, yaw, size_x, size_y, known, moveable, mass,
                                   restitution_coeff, friction_coeff, randomize, random_x_lower, random_x_upper,
                                   random_y_lower, random_y_upper, random_roll_lower, random_roll_upper,
                                   random_pitch_lower, random_pitch_upper, random_yaw_lower, random_yaw_upper,
                                   random_size_x_lower, random_size_x_upper, random_size_y_lower, random_size_y_upper,
                                   random_mass_lower, random_mass_upper, random_restitution_lower,
                                   random_restitution_upper, random_friction_lower, random_friction_upper);
      }
    } else {
      ROS_WARN("[Run] No obstacles specified");
    }

    // Load disturbance info.
    std::vector<DisturbanceInfo> disturbance_info;
    XmlRpc::XmlRpcValue disturbances;
    if (p_nh_.getParam("disturbances", disturbances)) {
      for (int i = 0; i < disturbances.size(); ++i) {
        XmlRpc::XmlRpcValue d = disturbances[i];
        double center_x = 0.0, center_y = 0.0;
        double size_x = 0.0, size_y = 0.0;
        double force_x = 0.0, force_y = 0.0;
        bool randomize = false;
        double random_center_x_lower = 0.0, random_center_x_upper = 0.0;
        double random_center_y_lower = 0.0, random_center_y_upper = 0.0;
        double random_size_x_lower = 0.0, random_size_x_upper = 0.0;
        double random_size_y_lower = 0.0, random_size_y_upper = 0.0;
        double random_force_angle_lower = 0.0, random_force_angle_upper = 0.0;
        double random_force_mag_lower = 0.0, random_force_mag_upper = 0.0;
        for (auto it = d.begin(); it != d.end(); ++it) {
          if (it->first == "center_x") {
            center_x = static_cast<double>(it->second);
          } else if (it->first == "center_y") {
            center_y = static_cast<double>(it->second);
          } else if (it->first == "size_x") {
            size_x = static_cast<double>(it->second);
          } else if (it->first == "size_y") {
            size_y = static_cast<double>(it->second);
          } else if (it->first == "force_x") {
            force_x = static_cast<double>(it->second);
          } else if (it->first == "force_y") {
            force_y = static_cast<double>(it->second);
          } else if (it->first == "randomize") {
            randomize = static_cast<bool>(it->second);
          } else if (it->first == "randomization_force_angle") {
            random_force_angle_lower = static_cast<double>(it->second[0]);
            random_force_angle_upper = static_cast<double>(it->second[1]);
          } else if (it->first == "randomization_force_mag") {
            random_force_mag_lower = static_cast<double>(it->second[0]);
            random_force_mag_upper = static_cast<double>(it->second[1]);
          }
        }
        ROS_INFO_STREAM("[Run]    Disturbance " << i << ": {center_x: " << center_x << ", center_y: " << center_y
                                                << ", size_x: " << size_x << ", size_y: " << size_y
                                                << ", force_x: " << force_x << ", force_y: " << force_y
                                                << ", randomize: " << (randomize ? "True" : "False") << "}");
        disturbance_info.emplace_back(center_x, center_y, size_x, size_y, force_x, force_y, randomize,
                                      random_center_x_lower, random_center_x_upper, random_center_y_lower,
                                      random_center_y_upper, random_size_x_lower, random_size_x_upper,
                                      random_size_y_lower, random_size_y_upper, random_force_angle_lower,
                                      random_force_angle_upper, random_force_mag_lower, random_force_mag_upper);
      }
    } else {
      ROS_WARN("[Run] No disturbances specified");
    }

    ellis_planner::Environment env;
    env.SetActions({ellis_planner::Environment::Action("+x", 0.1, 0.0, 0.0),
                    ellis_planner::Environment::Action("-x", -0.1, 0.0, 0.0),
                    ellis_planner::Environment::Action("+y", 0.0, 0.1, 0.0),
                    ellis_planner::Environment::Action("-y", 0.0, -0.1, 0.0)});
    env.GetRobotCollisionObject()->SetSizeX(robot_collision_size_x_);
    env.GetRobotCollisionObject()->SetSizeY(robot_collision_size_y_);
    env.SetBounds(-0.8, 0.8, -0.8, 0.8);
    ellis_planner::Search search(&env);

    // TODO(eratner) Get more parameters...

    // for (unsigned int i = 0; i < num_experiments; ++i) {
    for (unsigned int i = start_at_index; i < num_experiments + start_at_index; ++i) {
      ROS_INFO_STREAM("[Run] **Starting experiment " << (i - start_at_index) + 1 << "/" << num_experiments << "**");
      ROS_INFO_STREAM("[Run]   Index: " << i);
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

      start_x_ = exp_start_x;
      start_y_ = exp_start_y;
      goal_x_ = exp_goal_x;
      goal_y_ = exp_goal_y;

      std::vector<ObstacleInfo> exp_obstacle_info;
      std::vector<DisturbanceInfo> exp_disturbance_info;

      bool env_valid = false;
      int tries = 0;
      const int max_tries = 100;
      while (!env_valid && ros::ok()) {
        if (tries > max_tries) {
          ROS_WARN("[Run] Failed to find valid randomized environment!");
          break;
        }
        // // std::vector<ObstacleInfo> exp_obstacle_info = obstacle_info;
        // exp_obstacle_info = obstacle_info;
        // std::vector<ObstacleInfo> do_not_collide_with;
        // for (auto& o : exp_obstacle_info) {
        //   if (o.randomize_) o.Randomize(gen, do_not_collide_with);
        //   do_not_collide_with.push_back(o);
        // }

        // -------------------------------------------------------------------- //
        // TODO(eratner) Only for a specific scenario!
        // std::vector<ObstacleInfo>exp_obstacle_info = {obstacle_info[0]};
        exp_obstacle_info = {obstacle_info[0]};
        exp_obstacle_info[0].Randomize(gen);
        exp_obstacle_info.push_back(GetScenario2CornerObstacle(exp_obstacle_info[0]));
        for (int i = 1; i < obstacle_info.size(); ++i) {
          ObstacleInfo o = obstacle_info[i];
          o.Randomize(gen, exp_obstacle_info);
          exp_obstacle_info.push_back(o);
        }
        // -------------------------------------------------------------------- //

        // std::vector<DisturbanceInfo> exp_disturbance_info = disturbance_info;
        exp_disturbance_info = disturbance_info;
        for (auto& d : exp_disturbance_info) {
          if (d.randomize_) d.Randomize(gen);
        }

        // ------------------------------------------------------------------ //
        // Check if there exists a path through the environment; if not, re-randomize.
        std::vector<ellis_planner::CollisionObject::Ptr> c;
        for (const auto& o : exp_obstacle_info) {
          env.AddCollisionObject(
            new ellis_planner::RectangleCollisionObject("obs", o.x_, o.y_, o.yaw_, o.size_x_, o.size_y_));
        }
        for (const auto& d : exp_disturbance_info) {
          env.AddCollisionObject(
            new ellis_planner::RectangleCollisionObject("dist", d.center_x_, d.center_y_, 0.0, d.size_x_, d.size_y_));
        }
        env.SetGoal(exp_goal_x, exp_goal_y, 0.0);
        auto s = env.GetState(exp_start_x, exp_start_y, 0.0);
        std::vector<ellis_planner::State::Ptr> path;
        double path_cost;
        if (search.Run(s, path, path_cost)) {
          ROS_INFO("[Run] Environment ok!");
          env_valid = true;
        } else {
          ROS_WARN("[Run] Environment not valid! Randomizing again...");
        }
        env.Clear();
        // ------------------------------------------------------------------ //
        env_valid = true;
        tries++;
      }

      if (!env_valid) {
        ROS_ERROR("[Run] Environment not valid!");
        continue;
      }

      bool all_methods_ok = true;

      for (unsigned int j = 0; j < NUM_METHODS; ++j) {
        auto method = static_cast<Method>(j);
        bool run_method = p_nh_.param<bool>("run_method/" + ToString(method), true);
        if (!run_method) {
          ROS_WARN_STREAM("[Run] Skipping " << ToString(method) << "...");
          continue;
        }

        if (!all_methods_ok) {
          // TODO(eratner) Make this behavior a parameter
          ROS_WARN("[Run] Not all methods ok...skipping...");
          continue;
        }

        dynamic_reconfigure::DoubleParameter exec_error_penalty, robot_collision_size_x, robot_collision_size_y;
        exec_error_penalty.name = "exec_error_penalty";
        robot_collision_size_x.name = "robot_collision_size_x";
        robot_collision_size_y.name = "robot_collision_size_y";
        dynamic_reconfigure::BoolParameter use_control_level_penalty;
        use_control_level_penalty.name = "use_control_level_penalty";

        robot_collision_size_x.value = robot_collision_size_x_;
        robot_collision_size_y.value = robot_collision_size_y_;

        switch (method) {
          case OURS: {
            exec_error_penalty.value = 1000.0;
            use_control_level_penalty.value = true;
            break;
          }
          case REPLANNING: {
            exec_error_penalty.value = 0.0;
            use_control_level_penalty.value = false;
            break;
          }
          case CMAX: {
            exec_error_penalty.value = 1000.0;
            use_control_level_penalty.value = false;
            break;
          }
          default:
            break;
        }

        dynamic_reconfigure::Config conf;
        conf.doubles = {exec_error_penalty, robot_collision_size_x, robot_collision_size_y};
        conf.bools = {use_control_level_penalty};

        dynamic_reconfigure::ReconfigureRequest srv_req;
        srv_req.config = conf;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        ros::service::call("/cfg/planner_ellis/set_parameters", srv_req, srv_resp);

        Output output;
        output.method_ = method;
        output.random_seed_ = random_seed;
        output.start_x_ = exp_start_x;
        output.start_y_ = exp_start_y;
        output.start_yaw_ = exp_start_yaw;
        output.goal_x_ = exp_goal_x;
        output.goal_y_ = exp_goal_y;
        output.goal_yaw_ = exp_goal_yaw;
        output.execution_time_sec_ = 0.0;
        output.total_distance_ = 0.0;
        output.reached_goal_ = false;
        output.obstacles_ = exp_obstacle_info;
        output.disturbances_ = exp_disturbance_info;

        ClearObstacles();
        ClearDisturbances();

        // Add the barrier now, so the robot doesn't go off the edge while resetting.
        AddBarrier();

        // Move robot to the start pose.
        cfg_.Set<std::string>("planner", "trapezoidal");
        if (!cfg_.Reconfigure()) {
          ROS_ERROR("[Run] Could not change to trapezoidal planner!");
          continue;
        }
        state_ = REPLAN_NEEDED;
        while (ros::ok()) {
          if (state_ == REPLAN_NEEDED && AtStart()) {
            ROS_INFO("[Run] Done moving to the start!");
            break;
          } else if (state_ == REPLAN_NEEDED) {
            ROS_INFO("[Run] Replan needed...");
            if (!MoveTo(exp_start_x, exp_start_y, exp_start_yaw)) {
              ROS_ERROR_STREAM("[Run] Could not move to experiment start (" << exp_start_x << ", " << exp_start_y
                                                                            << ", " << exp_start_yaw << ")!");
              continue;
            }
            state_ = EXECUTING;
          } else if (state_ == ERROR) {
            ROS_ERROR("[Run] An error occured while moving to the start!");
            break;
          }
          ros::spinOnce();
          ros::Rate(5.0).sleep();
        }
        if (!AtStart()) {
          ROS_ERROR("[Run] Robot failed to reach start!");
          continue;
        }

        cfg_.Set<std::string>("planner", "ellis");
        if (!cfg_.Reconfigure()) {
          ROS_ERROR("[Run] Could not change to ellis planner!");
          continue;
        }

        // TODO(eratner) Reset everything in the planner
        ClearExecutionErrors();

        // Set up the known and unknown obstacles.
        for (const auto& o : exp_obstacle_info) {
          if (!AddObstacle(o)) ROS_WARN("[Run] Failed to add obstacle!");
          // if (!AddScenario2CornerObstacle(o)) ROS_WARN("[Run] Failed to add corner obstacle!");
        }

        for (const auto& d : exp_disturbance_info) {
          if (!AddDisturbance(d)) ROS_WARN("[Run] Failed to add disturbance!");
        }

        ROS_INFO("[Run] Waiting 5 seconds before starting...");
        ros::Duration(5.0).sleep();
        ROS_INFO("[Run] ...done!");
        // ROS_INFO("[Run] Press ENTER to start...");
        // std::cin.get();

        // Start writing to a bagfile.
        std::string path_to_bag = output_dir;
        if (path_to_bag.back() != '/') path_to_bag += '/';
        path_to_bag += ("exp_" + std::to_string(i) + "_" + ToString(method) + ".bag");
        bag_.open(path_to_bag, rosbag::bagmode::Write);
        output.path_to_bag_ = path_to_bag;

        std::string path_to_output_yaml = output_dir;
        if (path_to_output_yaml.back() != '/') path_to_output_yaml += '/';
        path_to_output_yaml += ("exp_" + std::to_string(i) + "_" + ToString(method) + ".yaml");

        for (const auto& msg : static_tf_msgs_) bag_.write("/tf_static", ros::Time::now(), msg);

        double last_x = 0.0, last_y = 0.0, last_z = 0.0, last_yaw = 0.0;
        GetPose(last_x, last_y, last_z, last_yaw);
        ros::Time time_of_last_motion = ros::Time::now();

        planner_prof_.clear();

        // Begin executing.
        ros::Time start_time = ros::Time::now();
        ellis_planner::Clock execution_time_clock;
        execution_time_clock.Start();
        state_ = REPLAN_NEEDED;
        ros::Rate rate(loop_rate_hz);
        while (ros::ok()) {
          double elapsed_time_sec = (ros::Time::now() - start_time).toSec();
          PublishElapsedTime(elapsed_time_sec, time_limit_sec);

          // if ((ros::Time::now() - start_time).toSec() >= time_limit_sec) {
          if (elapsed_time_sec >= time_limit_sec) {
            ROS_WARN("[Run] Time limit exceeded!");
            execution_time_clock.Stop();
            all_methods_ok = false;
            break;
          }

          // TODO(eratner) Make time to wait until robot stuck a parameter
          if ((ros::Time::now() - time_of_last_motion).toSec() >= 90.0) {
            ROS_WARN("[Run] Robot got stuck!");
            execution_time_clock.Stop();
            all_methods_ok = false;
            break;
          }

          switch (state_) {
            case REPLAN_NEEDED: {
              // ROS_INFO("[Run] Press ENTER to replan...");
              // std::cin.get();

              // Request to (re)plan and execute the path.
              if (!MoveTo(exp_goal_x, exp_goal_y, exp_goal_yaw)) {
                ROS_ERROR_STREAM("[Run] Move to (" << exp_goal_x << ", " << exp_goal_y << ", " << exp_goal_yaw
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
              ROS_INFO("[Run] Done!");
              execution_time_clock.Stop();
              break;
            }
            case ERROR: {
              ROS_ERROR("[Run] An error occured!");
              execution_time_clock.Stop();
              break;
            }
            default:
              break;
          }

          // Update the total distance moved.
          double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
          GetPose(x, y, z, yaw);
          double dist = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
          if (std::abs(dist) > 1e-3) time_of_last_motion = ros::Time::now();
          output.total_distance_ += dist;
          last_x = x;
          last_y = y;
          last_z = z;
          last_yaw = yaw;

          if (state_ == ERROR || state_ == DONE) break;

          ros::spinOnce();
          rate.sleep();
        }

        if (state_ == ERROR) {
          ROS_WARN("[Run] Experiment ended in an error! Moving on...");
          all_methods_ok = false;
        }

        output.execution_time_sec_ = execution_time_clock.GetElapsedTimeSec();
        output.reached_goal_ = AtGoal();

        ROS_INFO_STREAM("[Run] Writing output to " << path_to_output_yaml << "...");
        YAML::Node output_node = output.ToYaml();
        YAML::Node prof_node;
        for (const auto& p : planner_prof_) prof_node.push_back(YAML::Load(p));
        output_node["profiling"] = prof_node;
        std::ofstream fout(path_to_output_yaml);
        fout << output_node;
        // fout << output.ToYaml();

        // Stop writing to a bagfile.
        ROS_INFO("[Run] Writing to bag file...");
        bag_.close();
        ROS_INFO("[Run] ...done!");
      }
    }

    return true;
  }

 private:
  void ResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
        ROS_ERROR("[ResultCallback] Timeout on connect");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
        ROS_ERROR("[ResultCallback] Timeout on active");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
        ROS_ERROR("[ResultCallback] Timeout on response");
        break;
      case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
        ROS_ERROR("[ResultCallback] Timeout on deadline");
        break;
      case ff_util::FreeFlyerActionState::Enum::SUCCESS:
      case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
      case ff_util::FreeFlyerActionState::Enum::ABORTED: {
        ROS_INFO_STREAM("[ResultCallback] Result: " << result->fsm_result);
        switch (result->response) {
          case ff_msgs::MotionResult::ALREADY_THERE:
          case ff_msgs::MotionResult::SUCCESS:
          case ff_msgs::MotionResult::PREEMPTED: {
            if (AtGoal()) {
              // Experiment is done.
              ROS_INFO("[ResultCallback] Experiment is done!");
              state_ = DONE;
            } else {
              ROS_WARN("[ResultCallback] Not done yet! Replanning...");
              state_ = REPLAN_NEEDED;
            }
            break;
          }
            // TODO(eratner) What is the difference between POSITION_ENDPOINT and POSITION?
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION_ENDPOINT:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_ATTITUDE:
          case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION: {
            // A execution failure occurred!
            ROS_WARN("[ResultCallback] Something unexpected occurred!");

            ROS_INFO_STREAM("[ResultCallback] Publishing control history with " << control_history_.actual_pose.size()
                                                                                << " poses...");
            control_history_pub_.publish(control_history_);

            if (ReportExecutionError()) {
              ROS_INFO("[ResultCallback] Reported execution error...");
              state_ = REPLAN_NEEDED;
            } else {
              ROS_ERROR("[ResultCallback] Failed to report an execution error!");
              // state_ = ERROR;
              state_ = REPLAN_NEEDED;
            }

            break;
          }
          default:
            ROS_ERROR("[ResultCallback] Some other error has occured!");
            if (AtGoal()) {
              // TODO(eratner) Why does this happen sometimes?
              ROS_INFO("[ResultCallback] Actually, at the goal!");
              state_ = DONE;
            } else {
              // state_ = ERROR;
              state_ = REPLAN_NEEDED;
            }
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
      ROS_DEBUG_STREAM("    Actual pos: (" << actual_x << ", " << actual_y << ", " << actual_z
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
      ROS_ERROR_STREAM("[GetPose] Error: " << ex.what());
      return false;
    }

    x = world_to_body.transform.translation.x;
    y = world_to_body.transform.translation.y;
    z = world_to_body.transform.translation.z;
    yaw = tf2::getYaw(world_to_body.transform.rotation);
    return true;
  }

  bool AtStart(double start_dist_tol = 0.05) {
    double curr_x = 0.0, curr_y = 0.0, curr_z = 0.0, curr_yaw = 0.0;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_WARN_STREAM("[AtStart] Could not check if at start, because could not get current pose!");
      return false;
    }

    double dist = std::sqrt(std::pow(start_x_ - curr_x, 2) + std::pow(start_y_ - curr_y, 2));
    return (dist < start_dist_tol);
  }

  // bool AtGoal(double goal_dist_tol = 0.15) {
  bool AtGoal(double goal_dist_tol = 0.20) {
    double curr_x = 0.0, curr_y = 0.0, curr_z = 0.0, curr_yaw = 0.0;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_WARN_STREAM("[AtGoal] Could not check if at goal, because could not get current pose!");
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
      ROS_ERROR("[ReportExecutionError] Failed to find root of error!");
      return false;
    }

    ROS_WARN_STREAM("[ReportExecutionError] Found root of error at index " << index << " (history size is "
                                                                           << control_feedback_history_.size() << ")");
    const auto& fb = control_feedback_history_[index];
    ROS_WARN_STREAM("[ReportExecutionError]   pos err: "
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
      ROS_ERROR("[ReportExecutionError] Failed to call service to report an execution error");
      return false;
    }

    return true;
  }

  bool MoveTo(double x, double y, double yaw = 0.0) {
    // Get the robot's current pose.
    double curr_x, curr_y, curr_z, curr_yaw;
    if (!GetPose(curr_x, curr_y, curr_z, curr_yaw)) {
      ROS_ERROR("[MoveTo] Failed to get robot's current pose!");
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
    ROS_INFO_STREAM("[MoveTo] Sending motion goal with position ("
                    << state.pose.position.x << ", " << state.pose.position.y << ", " << state.pose.position.z
                    << ") and yaw " << yaw << "...");
    if (!mobility_client_.SendGoal(goal)) {
      ROS_ERROR("[MoveTo] Mobility client did not accept goal");
      return false;
    }

    return true;
  }

  bool ClearObstacles() {
    std_srvs::Trigger srv;
    bool clear_planner_ok = clear_planner_obstacles_client_.call(srv);
    bool clear_sim_ok = clear_simulator_obstacles_client_.call(srv);
    return (clear_planner_ok && clear_sim_ok);
  }

  bool AddObstacle(const ObstacleInfo& info, double size_z = 0.75) {
    ellis_planner::AddObstacle add_obs;
    add_obs.request.type = info.type_;
    add_obs.request.name = info.name_;
    add_obs.request.pose.position.x = info.x_;
    add_obs.request.pose.position.y = info.y_;
    add_obs.request.pose.position.z = -0.674614;  // TODO(eratner) Fix this
    tf2::Quaternion orien;
    orien.setRPY(info.roll_, info.pitch_, info.yaw_);
    add_obs.request.pose.orientation.x = orien.x();
    add_obs.request.pose.orientation.y = orien.y();
    add_obs.request.pose.orientation.z = orien.z();
    add_obs.request.pose.orientation.w = orien.w();
    add_obs.request.size.x = info.size_x_;
    add_obs.request.size.y = info.size_y_;
    add_obs.request.size.z = size_z;
    add_obs.request.moveable = info.moveable_;
    add_obs.request.mass = info.mass_;
    add_obs.request.restitution_coeff = info.restitution_coeff_;
    add_obs.request.friction_coeff = info.friction_coeff_;

    bool add_sim_ok = add_simulator_obstacle_client_.call(add_obs);
    bool add_planner_ok = true;
    if (info.known_) {
      // Add a known obstacle.
      // TODO(eratner) Because the robot's likely to hit the obstacle, this could cause issues for collision checking,
      // so shrink the obstacle a bit (not the cleanest solution, but a difficult situation overall).
      // add_obs.request.size.x -= 0.01;
      // add_obs.request.size.y -= 0.01;
      add_obs.request.size.x -= 0.015;
      add_obs.request.size.y -= 0.015;
      // add_obs.request.size.x -= 0.02;
      // add_obs.request.size.y -= 0.02;
      add_planner_ok = add_planner_obstacle_client_.call(add_obs);
    }

    return (add_sim_ok && add_planner_ok);
  }

  ObstacleInfo GetScenario2CornerObstacle(const ObstacleInfo& info) {
    ObstacleInfo corner_obs;
    corner_obs.type_ = "rectangle";
    corner_obs.name_ = "corner";
    corner_obs.x_ = info.x_ + 0.5 * info.size_x_ + 0.05;
    corner_obs.y_ = info.y_ + 0.075;
    corner_obs.yaw_ = 0;
    corner_obs.size_x_ = 0.1;
    corner_obs.size_y_ = info.size_y_ + 0.15;
    corner_obs.known_ = info.known_;
    corner_obs.moveable_ = false;
    return corner_obs;
  }

  // bool AddBarrier() {
  //   ObstacleInfo barrier_obs;
  //   barrier_obs.type_ = "rectangle";
  //   barrier_obs.name_ = "barrier";
  //   barrier_obs.x_ = 0;
  //   barrier_obs.y_ = 1.05;
  //   barrier_obs.yaw_ = 0;
  //   barrier_obs.size_x_ = 2.0;
  //   barrier_obs.size_y_ = 0.1;
  //   barrier_obs.known_ = true;
  //   barrier_obs.moveable_ = false;
  //   return AddObstacle(barrier_obs);
  // }

  bool AddBarrier() {
    bool all_ok = true;

    {
      ObstacleInfo barrier_obs;
      barrier_obs.type_ = "rectangle";
      barrier_obs.name_ = "barrier1";
      barrier_obs.x_ = 0;
      barrier_obs.y_ = 1.05;
      barrier_obs.yaw_ = 0;
      barrier_obs.size_x_ = 2.0;
      barrier_obs.size_y_ = 0.1;
      barrier_obs.known_ = true;
      barrier_obs.moveable_ = false;
      if (!AddObstacle(barrier_obs)) all_ok = false;
    }

    {
      ObstacleInfo barrier_obs;
      barrier_obs.type_ = "rectangle";
      barrier_obs.name_ = "barrier2";
      barrier_obs.x_ = 0;
      barrier_obs.y_ = -1.05;
      barrier_obs.yaw_ = 0;
      barrier_obs.size_x_ = 2.0;
      barrier_obs.size_y_ = 0.1;
      barrier_obs.known_ = true;
      barrier_obs.moveable_ = false;
      if (!AddObstacle(barrier_obs)) all_ok = false;
    }

    {
      ObstacleInfo barrier_obs;
      barrier_obs.type_ = "rectangle";
      barrier_obs.name_ = "barrier3";
      barrier_obs.x_ = 1.05;
      barrier_obs.y_ = 0;
      barrier_obs.yaw_ = 0;
      barrier_obs.size_x_ = 0.1;
      barrier_obs.size_y_ = 2.0;
      barrier_obs.known_ = true;
      barrier_obs.moveable_ = false;
      if (!AddObstacle(barrier_obs)) all_ok = false;
    }

    {
      ObstacleInfo barrier_obs;
      barrier_obs.type_ = "rectangle";
      barrier_obs.name_ = "barrier4";
      barrier_obs.x_ = -1.05;
      barrier_obs.y_ = 0;
      barrier_obs.yaw_ = 0;
      barrier_obs.size_x_ = 0.1;
      barrier_obs.size_y_ = 2.0;
      barrier_obs.known_ = true;
      barrier_obs.moveable_ = false;
      if (!AddObstacle(barrier_obs)) all_ok = false;
    }

    return all_ok;
  }

  bool AddDisturbance(const DisturbanceInfo& info) {
    ellis_planner::AddDisturbance add_dist;
    add_dist.request.center.x = info.center_x_;
    add_dist.request.center.y = info.center_y_;
    add_dist.request.size.x = info.size_x_;
    add_dist.request.size.y = info.size_y_;
    add_dist.request.wrench.force.x = info.force_x_;
    add_dist.request.wrench.force.y = info.force_y_;
    return add_disturbance_client_.call(add_dist);
  }

  bool ClearDisturbances() {
    std_srvs::Trigger srv;
    return clear_disturbances_client_.call(srv);
  }

  bool ClearExecutionErrors() {
    std_srvs::Trigger srv;
    return clear_execution_errors_client_.call(srv);
  }

  void PublishElapsedTime(double elapsed_time_sec, double max_time_sec = 10000.0) {
    visualization_msgs::Marker time_msg;
    time_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    time_msg.ns = "/experiments/elapsed_time_sec";
    time_msg.id = 1;
    time_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    time_msg.pose.position.x = 0.0;
    time_msg.pose.position.y = 0.0;
    time_msg.pose.position.z = -0.8;
    if (elapsed_time_sec >= max_time_sec) {
      time_msg.color.r = 1.0;
      time_msg.color.g = 0.0;
      time_msg.color.b = 0.0;
    } else if (max_time_sec - elapsed_time_sec <= 60.0) {
      time_msg.color.r = 0.0;
      time_msg.color.g = 0.5;
      time_msg.color.b = 0.5;
    } else {
      time_msg.color.r = 0.0;
      time_msg.color.g = 0.0;
      time_msg.color.b = 0.0;
    }
    time_msg.color.a = 1.0;
    time_msg.scale.z = 0.08;

    std::ostringstream out;
    out.precision(5);
    out << elapsed_time_sec;

    time_msg.text = out.str();
    vis_pub_.publish(time_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle p_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> mobility_client_;
  ff_util::ConfigClient cfg_;
  ros::ServiceClient set_zones_client_;
  ros::ServiceClient add_simulator_obstacle_client_;
  ros::ServiceClient clear_simulator_obstacles_client_;
  ros::ServiceClient add_planner_obstacle_client_;
  ros::ServiceClient clear_planner_obstacles_client_;
  ros::ServiceClient add_disturbance_client_;
  ros::ServiceClient clear_disturbances_client_;
  ros::ServiceClient report_execution_error_client_;
  ros::ServiceClient clear_execution_errors_client_;
  boost::circular_buffer<ff_msgs::ControlFeedback> control_feedback_history_;
  ellis_planner::ControlHistory control_history_;
  ros::Publisher control_history_pub_;
  State state_;

  ros::Publisher vis_pub_;

  double robot_collision_size_x_;
  double robot_collision_size_y_;

  // Experiment instance info.
  double start_x_;
  double start_y_;
  double goal_x_;
  double goal_y_;

  rosbag::Bag bag_;
  std::vector<ros::Subscriber> subs_;
  std::vector<tf2_msgs::TFMessage> static_tf_msgs_;
  std::vector<std::string> planner_prof_;
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
