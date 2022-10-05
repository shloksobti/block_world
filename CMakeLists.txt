cmake_minimum_required(VERSION 3.0.2)
project(block_world)

list(APPEND CATKIN_SOURCES
  actionlib
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  robowflex_library
  robowflex_movegroup
  robowflex_ompl
  message_generation
  roscpp
  std_msgs
  trajectory_msgs
  visualization_msgs
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

include_directories(
  include
  include/block_world
  ${catkin_INCLUDE_DIRS}
)

add_service_files(DIRECTORY srv
  FILES
  BlankSrv.srv
  )

add_action_files(DIRECTORY action
  FILES
  StackBlocks.action
  )

generate_messages(DEPENDENCIES 
  std_msgs
  actionlib_msgs
  )

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS ${CATKIN_SOURCES} 
DEPENDS 
)

add_library(StackBlocks src/StackBlocks.cpp include/block_world/StackBlocks.h)
target_link_libraries(StackBlocks ${catkin_LIBRARIES})

add_executable(block_world_node scripts/block_world.cpp)
target_link_libraries(block_world_node ${catkin_LIBRARIES} StackBlocks)
