project(discrepancy_planner)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES discrepancy_planner
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    choreographer
    ff_msgs
    ff_util
    angles
    visualization_msgs
    astrobee_search_based_planning
)

# TODO This is messy, clean it up
include_directories(
  ../ellis_util/include
  ../astrobee_planning_research/astrobee_search_based_planning/include
)

create_library(
  TARGET discrepancy_planner
  LIBS ${catkin_LIBRARIES} choreographer jsonloader ff_flight ff_nodelet ellis_util astrobee_search_based_planning
  INC ${catkin_INCLUDE_DIRS}
  DEPS choreographer ellis_util
)

create_tool_targets(
  DIR tools
  LIBS ${catkin_LIBRARIES} ${PROJECT_NAME} ff_nodelet ff_flight config_client jsonloader choreographer astrobee_search_based_planning
  INC ${catkin_INCLUDE_DIRS}
  DEPS choreographer
)

install_launch_files()
