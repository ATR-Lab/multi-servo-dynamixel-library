execute_process(COMMAND "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
