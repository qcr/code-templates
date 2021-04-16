cmake_minimum_required(VERSION 3.0.2)
project(PACKAGE_NAME)

TEMPLATE_START ADD_CPP_NODE
add_compile_options(-std=c++11)
TEMPLATE_END

find_package(catkin REQUIRED
  TEMPLATE_START ADD_MSGS ADD_SERVICES ADD_ACTIONS
  COMPONENTS message_generation
  TEMPLATE_END
  )

TEMPLATE_START ADD_PY_NODE
catkin_python_setup()
TEMPLATE_END

TEMPLATE_START ADD_MSGS
add_message_files(
  FILES
  Message1.msg
  )
TEMPLATE_END

TEMPLATE_START ADD_SERVICES
add_service_files(
  FILES
  Service1.srv
  )
TEMPLATE_END

TEMPLATE_START ACC_ACTIONS
add_action_files(
  FILES
  Action1.action
  Action2.action
  )
TEMPLATE_END

TEMPLATE_START ADD_MSGS ADD_SERVICES ADD_ACTIONS
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  )
TEMPLATE_END

catkin_package(
  TEMPLATE_START ADD_MSGS ADD_SERVICES ADD_ACTIONS
  CATKIN_DEPENDS message_runtime
  TEMPLATE_END
  )

TEMPLATE_START ADD_CPP_NODE
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ros_package.cpp
  )
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  )

add_executable(${PROJECT_NAME}_node src/ros_package_node.cpp)
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME node PREFIX ""
  )
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )
TEMPLATE_END

TEMPLATE_START ADD_PY_NODE
catkin_install_python(PROGRAMS
  scripts/my_python_script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
TEMPLATE_END

TEMPLATE_START ADD_CPP_NODE
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
TEMPLATE_END
