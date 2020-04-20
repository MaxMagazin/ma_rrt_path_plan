cmake_minimum_required(VERSION 2.8.12)
project(vehicle_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Command.msg
  TrackCone.msg
  Track.msg
  Waypoint.msg
  WaypointsArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
