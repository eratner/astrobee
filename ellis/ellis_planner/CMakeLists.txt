cmake_minimum_required(VERSION 3.0)
project(ellis_planner)

add_compile_options(-std=c++14)

find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  choreographer
  ff_util
  ff_msgs
  tf2_ros
  tf2
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES ellis_planner
  CATKIN_DEPENDS
  roscpp
  ff_util
  ff_msgs
  choreographer
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# TODO C++ libraries
# TODO Install C++ libraries

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
