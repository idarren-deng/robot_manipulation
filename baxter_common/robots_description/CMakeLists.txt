cmake_minimum_required(VERSION 2.8.3)
project(robots_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir meshes urdf robots)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
