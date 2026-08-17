cmake_minimum_required(VERSION 3.16)
project(legged_state_estimator_ros2)

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
  legged_state_estimator

  rclcpp
  rclcpp_lifecycle

  tf2_eigen

  sensor_msgs
  geometry_msgs
  contact_msgs
  legged_locomotion_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Legged MPC controller
add_executable(legged_state_estimator_node
  src/LeggedStateEstimatorNode.cpp
  src/main.cpp
)
target_include_directories(legged_state_estimator_node
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)
ament_target_dependencies(legged_state_estimator_node ${THIS_PACKAGE_DEPENDS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include test
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)


#############
## Install ##
#############

install(TARGETS legged_state_estimator_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
