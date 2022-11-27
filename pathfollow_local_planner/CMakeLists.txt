cmake_minimum_required(VERSION 3.0.2)
project(pathfollow_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/PathFollowLocalPlanner.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES pathfollow_local_planner
   CATKIN_DEPENDS base_local_planner costmap_2d dynamic_reconfigure geometry_msgs nav_core nav_msgs pluginlib roscpp tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pathfollow_local_planner src/pathfollow_local_planner.cpp)
add_dependencies(pathfollow_local_planner pathfollow_local_planner_gencfg)
target_link_libraries(pathfollow_local_planner ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES pathfollow_local_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)