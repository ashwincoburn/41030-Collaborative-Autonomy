execute_process(COMMAND "/home/arcbash/catkin_ws/src/build/turtlebot3_autorace/turtlebot3_autorace_core/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/arcbash/catkin_ws/src/build/turtlebot3_autorace/turtlebot3_autorace_core/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
