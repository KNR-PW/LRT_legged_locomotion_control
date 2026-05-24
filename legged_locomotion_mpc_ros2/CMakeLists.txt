cmake_minimum_required(VERSION 3.16)
project(legged_locomotion_mpc_ros2)

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
  legged_locomotion_mpc
  contact_msgs
  legged_locomotion_msgs

  rclcpp
  tf2_ros
  sensor_msgs
  visualization_msgs
  geometry_msgs
  trajectory_msgs
  ocs2_ros_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  src/visualization/LeggedVisualizer.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

# Meldog demos
add_executable(${PROJECT_NAME}_meldog_demo_planar_terrain
  src/demos/meldog_demo/meldog_demo_planar_terrain.cpp
)
target_link_libraries(${PROJECT_NAME}_meldog_demo_planar_terrain ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_meldog_demo_planar_terrain ${THIS_PACKAGE_DEPENDS})

add_executable(${PROJECT_NAME}_meldog_demo_segmented_terrain
  src/demos/meldog_demo/meldog_demo_segmented_terrain.cpp
)
target_link_libraries(${PROJECT_NAME}_meldog_demo_segmented_terrain ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_meldog_demo_segmented_terrain ${THIS_PACKAGE_DEPENDS})

# Melman demo
add_executable(${PROJECT_NAME}_melman_demo
  src/demos/melman_demo/melman_demo.cpp
)
target_link_libraries(${PROJECT_NAME}_melman_demo ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_melman_demo ${THIS_PACKAGE_DEPENDS})

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

install(TARGETS ${PROJECT_NAME}_meldog_demo_planar_terrain
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_meldog_demo_segmented_terrain
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_melman_demo
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
