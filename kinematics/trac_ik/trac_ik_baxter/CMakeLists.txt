cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_baxter)

find_package(catkin REQUIRED COMPONENTS
  trac_ik_lib
  eigen_conversions
  message_generation
  sensor_msgs
  geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
)

add_service_files(
  FILES
  GetConstrainedPositionIK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(trac_ik_server src/ik_server.cpp)
target_link_libraries(trac_ik_server ${catkin_LIBRARIES})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    trac_ik_lib
    message_runtime
)

install(TARGETS trac_ik_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
