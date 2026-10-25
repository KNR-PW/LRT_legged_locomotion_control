cmake_minimum_required(VERSION 3.8)
project(terrain_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

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
    ocs2_core
    ocs2_robotic_tools
    grid_map_sdf
    pcl_ros
    convex_plane_decomposition_ros
)

set(FLAGS
        ${OCS2_CXX_FLAGS}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(${PROJECT_NAME}
  src/core/ConvexTerrain.cpp
  src/core/TerrainPlane.cpp
  src/core/TerrainModel.cpp

  src/planar_model/PlanarSignedDistanceField.cpp
  src/planar_model/PlanarTerrainModel.cpp
  src/planar_model/PlanarFactoryFunctions.cpp

  src/segmented_planes_model/SegmentedPlanesSignedDistanceField.cpp
  src/segmented_planes_model/SegmentedPlanesTerrainModel.cpp
  src/segmented_planes_model/SegmentedPlanesFactoryFunctions.cpp
  src/segmented_planes_model/SegmentedPlanesGridMapUtils.cpp

)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}_terrain test/testTerrainPlane.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_terrain ${THIS_PACKAGE_DEPENDS})
  target_link_libraries(test_${PROJECT_NAME}_terrain ${PROJECT_NAME})


  ament_add_gtest(test_${PROJECT_NAME}_convexTerrain test/testConvexTerrain.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_convexTerrain ${THIS_PACKAGE_DEPENDS})
  target_link_libraries(test_${PROJECT_NAME}_convexTerrain ${PROJECT_NAME})

  ament_add_gtest(test_${PROJECT_NAME}_planarFactory test/testPlanarFactoryFunctions.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_planarFactory ${THIS_PACKAGE_DEPENDS})
  target_link_libraries(test_${PROJECT_NAME}_planarFactory ${PROJECT_NAME})

  ament_add_gtest(test_${PROJECT_NAME}_planarTerrainModel test/testPlanarTerrainModel.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_planarTerrainModel ${THIS_PACKAGE_DEPENDS})
  target_link_libraries(test_${PROJECT_NAME}_planarTerrainModel ${PROJECT_NAME})

  ament_add_gtest(test_${PROJECT_NAME}_segmentedPlanesGridMapUtils test/testSegmentedPlanesGridMapUtils.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_segmentedPlanesGridMapUtils ${THIS_PACKAGE_DEPENDS})
  target_link_libraries(test_${PROJECT_NAME}_segmentedPlanesGridMapUtils ${PROJECT_NAME})

  ament_add_gtest(test_${PROJECT_NAME}_segmentedPlanesSignedDistanceField test/testSegmentedPlanesSignedDistanceField.cpp)
  ament_target_dependencies(test_${PROJECT_NAME}_segmentedPlanesSignedDistanceField ${THIS_PACKAGE_DEPENDS})
  target_link_libraries(test_${PROJECT_NAME}_segmentedPlanesSignedDistanceField ${PROJECT_NAME})

endif()

ament_package()