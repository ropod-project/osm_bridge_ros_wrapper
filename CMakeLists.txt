cmake_minimum_required(VERSION 2.8.6)

project(osm_bridge_ros_wrapper)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ropod_wm_mediator/cmake")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(INSTALL_DIR /usr/lib CACHE PATH "Installation directory for mediator")



########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg 
  actionlib_msgs 
  actionlib
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg/osm
  FILES
  Node.msg
  Way.msg
  Tag.msg
  Relation.msg
  Member.msg
)

add_message_files(
  DIRECTORY msg/planner
  FILES
  PlannerArea.msg
)

add_message_files(
  DIRECTORY msg/wm
  FILES
  Area.msg
  Building.msg
  Connection.msg
  Corridor.msg
  Door.msg
  Elevator.msg
  Feature.msg
  Floor.msg
  LocalArea.msg
  Room.msg
  Side.msg
  Stairs.msg
  Wall.msg
)


add_action_files(
  DIRECTORY action
  FILES
  WMQuery.action
  OSMQuery.action
  PathPlanner.action
  GridMapGenerator.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime std_msgs actionlib_msgs
)
