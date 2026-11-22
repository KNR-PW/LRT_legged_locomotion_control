cmake_minimum_required(VERSION 3.16)
project(legged_whole_body_control_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Resolve for the package path at compile time.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h" @ONLY
)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
  rclcpp
  rclcpp_lifecycle

  tf2_eigen

  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_pinocchio_interface
  floating_base_model
  legged_whole_body_control

  sensor_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  
  ocs2_msgs
  ocs2_ros_interfaces
  contact_msgs
  legged_locomotion_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Legged WBC controller
add_executable(legged_whole_body_controller_node
  src/LeggedWholeBodyControllerNode.cpp
  src/MpcPolicyEvaluator.cpp
  src/main.cpp
)
target_include_directories(legged_whole_body_controller_node
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)
ament_target_dependencies(legged_whole_body_controller_node ${THIS_PACKAGE_DEPENDS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target legged_whole_body_controller_node")
    add_clang_tooling(
            TARGETS legged_whole_body_controller_node
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS legged_whole_body_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Launch files, ocs2 configs and ROS parameters
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

