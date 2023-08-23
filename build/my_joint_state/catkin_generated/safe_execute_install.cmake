execute_process(COMMAND "/home/victor/catkin_ws/build/my_joint_state/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/victor/catkin_ws/build/my_joint_state/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
