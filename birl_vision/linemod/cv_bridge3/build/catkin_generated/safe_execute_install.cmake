execute_process(COMMAND "/home/darren/baxter_ws/src/birl_vision/linemod/cv_bridge3/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/darren/baxter_ws/src/birl_vision/linemod/cv_bridge3/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
