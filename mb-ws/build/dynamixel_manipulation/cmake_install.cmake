# Install script for directory: /home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_manipulation/srv" TYPE FILE FILES
    "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv"
    "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv"
    "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_manipulation/cmake" TYPE FILE FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/installspace/dynamixel_manipulation-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/include/dynamixel_manipulation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/share/roseus/ros/dynamixel_manipulation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/share/common-lisp/ros/dynamixel_manipulation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/share/gennodejs/ros/dynamixel_manipulation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/lib/python2.7/dist-packages/dynamixel_manipulation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/lib/python2.7/dist-packages/dynamixel_manipulation" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/lib/python2.7/dist-packages/dynamixel_manipulation" FILES_MATCHING REGEX "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/devel/lib/python2.7/dist-packages/dynamixel_manipulation/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/installspace/dynamixel_manipulation.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_manipulation/cmake" TYPE FILE FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/installspace/dynamixel_manipulation-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_manipulation/cmake" TYPE FILE FILES
    "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/installspace/dynamixel_manipulationConfig.cmake"
    "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation/catkin_generated/installspace/dynamixel_manipulationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_manipulation" TYPE FILE FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dynamixel_manipulation" TYPE PROGRAM FILES "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/scripts/my_python_script")
endif()

