cmake_minimum_required(VERSION 3.0)
project(ellis_planner)

add_compile_options(-std=c++14)

find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_util
  ff_msgs
  tf2_ros
  tf2
  nodelet
  pluginlib
  choreographer
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ellis_planner
  CATKIN_DEPENDS
    roscpp
    ff_util
    ff_msgs
    tf2_ros
    tf2
    nodelet
    pluginlib
    choreographer
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ellis_planner
  src/planner_interface.cpp
  src/polynomial.cpp
  src/state.cpp
)
add_dependencies(ellis_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(ellis_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(exp tools/experiment_manager.cpp)
add_dependencies(exp ${catkin_EXPORTED_TARGETS})
target_link_libraries(exp ${catkin_LIBRARIES})
install(TARGETS exp DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/exp share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
