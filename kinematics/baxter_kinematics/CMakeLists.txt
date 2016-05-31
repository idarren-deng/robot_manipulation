cmake_minimum_required(VERSION 2.8.3)
project(baxter_kinematics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  trac_ik_baxter
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
