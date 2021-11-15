# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dynamixel_manipulation: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dynamixel_manipulation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" NAME_WE)
add_custom_target(_dynamixel_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamixel_manipulation" "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" ""
)

get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" NAME_WE)
add_custom_target(_dynamixel_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamixel_manipulation" "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" ""
)

get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" NAME_WE)
add_custom_target(_dynamixel_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamixel_manipulation" "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_cpp(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_cpp(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_manipulation
)

### Generating Module File
_generate_module_cpp(dynamixel_manipulation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_manipulation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dynamixel_manipulation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dynamixel_manipulation_generate_messages dynamixel_manipulation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_cpp _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_cpp _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_cpp _dynamixel_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamixel_manipulation_gencpp)
add_dependencies(dynamixel_manipulation_gencpp dynamixel_manipulation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamixel_manipulation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_eus(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_eus(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamixel_manipulation
)

### Generating Module File
_generate_module_eus(dynamixel_manipulation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamixel_manipulation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dynamixel_manipulation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dynamixel_manipulation_generate_messages dynamixel_manipulation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_eus _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_eus _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_eus _dynamixel_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamixel_manipulation_geneus)
add_dependencies(dynamixel_manipulation_geneus dynamixel_manipulation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamixel_manipulation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_lisp(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_lisp(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_manipulation
)

### Generating Module File
_generate_module_lisp(dynamixel_manipulation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_manipulation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dynamixel_manipulation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dynamixel_manipulation_generate_messages dynamixel_manipulation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_lisp _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_lisp _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_lisp _dynamixel_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamixel_manipulation_genlisp)
add_dependencies(dynamixel_manipulation_genlisp dynamixel_manipulation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamixel_manipulation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_nodejs(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_nodejs(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamixel_manipulation
)

### Generating Module File
_generate_module_nodejs(dynamixel_manipulation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamixel_manipulation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dynamixel_manipulation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dynamixel_manipulation_generate_messages dynamixel_manipulation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_nodejs _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_nodejs _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_nodejs _dynamixel_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamixel_manipulation_gennodejs)
add_dependencies(dynamixel_manipulation_gennodejs dynamixel_manipulation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamixel_manipulation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_py(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation
)
_generate_srv_py(dynamixel_manipulation
  "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation
)

### Generating Module File
_generate_module_py(dynamixel_manipulation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dynamixel_manipulation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dynamixel_manipulation_generate_messages dynamixel_manipulation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_py _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/StopController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_py _dynamixel_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/srv/RestartController.srv" NAME_WE)
add_dependencies(dynamixel_manipulation_generate_messages_py _dynamixel_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamixel_manipulation_genpy)
add_dependencies(dynamixel_manipulation_genpy dynamixel_manipulation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamixel_manipulation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_manipulation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dynamixel_manipulation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamixel_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamixel_manipulation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dynamixel_manipulation_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_manipulation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dynamixel_manipulation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamixel_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamixel_manipulation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dynamixel_manipulation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_manipulation/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dynamixel_manipulation_generate_messages_py std_msgs_generate_messages_py)
endif()
