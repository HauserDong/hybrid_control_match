cmake_minimum_required(VERSION 2.8.3)
project(mbot_explore)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  map_msgs
  move_base_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
)


add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package( 

CATKIN_DEPENDS 
  message_runtime
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)




###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(global_rrt_detector
	src/global_rrt_detector.cpp 
	src/functions.cpp 
	src/mtrand.cpp
	)
target_link_libraries(global_rrt_detector ${catkin_LIBRARIES})
add_dependencies(global_rrt_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(local_rrt_detector ${catkin_LIBRARIES})
add_executable(mbot_explore
  src/costmap_client.cpp
  src/explore.cpp
  src/frontier_search.cpp
)
add_dependencies(mbot_explore ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mbot_explore ${catkin_LIBRARIES})
#############
## Install ##
#############

