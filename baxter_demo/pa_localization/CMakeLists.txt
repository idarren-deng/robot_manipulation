cmake_minimum_required(VERSION 2.8.3)
project(pa_localization)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  rospy
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES pa_location.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
