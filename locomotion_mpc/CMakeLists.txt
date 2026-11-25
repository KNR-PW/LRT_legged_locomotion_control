cmake_minimum_required(VERSION 3.16)
project(legged_locomotion_mpc)

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
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_sphere_approximation
  floating_base_model
  terrain_model
  multi_end_effector_kinematics
)

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  src/common/Utils.cpp
  src/common/ModelSettings.cpp
  src/common/ModelHelperFunctions.cpp

  src/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.cpp
  src/torque_approx/PinocchioTorqueApproximationCppAd.cpp

  src/collision/CollisionSettings.cpp
  src/collision/PinocchoCollisionInterface.cpp
  src/collision/PinocchioForwardCollisionKinematicsCppAd.cpp

  src/locomotion/GaitParameters.cpp
  src/locomotion/ModeSequenceTemplate.cpp
  src/locomotion/GaitPlanner.cpp
  src/locomotion/ModeDynamicSequenceTemplate.cpp
  src/locomotion/GaitDynamicPhaseController.cpp
  src/locomotion/CubicSpline.cpp
  src/locomotion/QuinticSplineSwing.cpp
  src/locomotion/SwingSpline3d.cpp
  src/locomotion/SingleLegLogic.cpp
  src/locomotion/FootPhase.cpp
  src/locomotion/OverExtensionPenalty.cpp
  src/locomotion/SwingTrajectoryPlanner.cpp

  src/trajectory_planners/BaseTrajectoryPlanner.cpp
  src/trajectory_planners/JointTrajectoryPlanner.cpp
  src/trajectory_planners/ContactForceWrenchTrajectoryPlanner.cpp

  src/weight_compensation/PinocchioWeightCompensator.cpp

  src/synchronization/GaitSynchronizedModule.cpp
  src/synchronization/DisturbanceSynchronizedModule.cpp

  src/reference_manager/LeggedReferenceManager.cpp

  src/precomputation/LeggedPrecomputation.cpp

  src/dynamics/LeggedDynamicsAD.cpp

  src/constraint/Zero3DofVelocityConstraint.cpp
  src/constraint/Zero6DofVelocityConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/ZeroWrenchConstraint.cpp
  src/constraint/NormalVelocityConstraint.cpp

  src/soft_constraint/JointTorqueLimitsSoftConstraint.cpp
  src/soft_constraint/JointLimitsSoftConstraint.cpp
  src/soft_constraint/TerrainAvoidanceSoftConstraint.cpp
  src/soft_constraint/SelfCollisionAvoidanceSoftConstraint.cpp
  src/soft_constraint/EndEffectorPlacementSoftConstraint.cpp
  src/soft_constraint/NormalOrientationSoftConstraint.cpp
  src/soft_constraint/ForceFrictionConeSoftConstraint.cpp
  src/soft_constraint/WrenchFrictionConeSoftConstraint.cpp

  src/initialization/LeggedInitializer.cpp

  src/cost/TrajectoryTrackingCost.cpp
  src/cost/TerminalTrackingCost.cpp
  src/cost/JointTorqueCost.cpp

  src/robot_interface/LeggedInterface.cpp
  src/robot_interface/LeggedLoopshapingInterface.cpp
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

  ###########
  ## Tests ##
  ###########
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_common
        test/common/UtilsTest.cpp
        test/common/ModelHelperFunctionsTest.cpp
        test/common/EulerTransformsTest.cpp
        test/common/ModelSettingsTest.cpp
        test/common/MpcSolutionMonitorTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_common ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_common PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_common ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_common PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_locomotion
        test/locomotion/GaitParametersTest.cpp
        test/locomotion/GaitCommonTest.cpp
        test/locomotion/ModeDynamicSequenceTemplateTest.cpp
        test/locomotion/GaitPlannerTest.cpp
        test/locomotion/GaitPlannerRateLimitTest.cpp
        test/locomotion/GaitPlannerRephasingTest.cpp
        test/locomotion/GaitDynamicPhaseControllerTest.cpp
        test/locomotion/FootPhaseTest.cpp
        test/locomotion/SingleLegLogicTest.cpp
        test/locomotion/OverExtensionPenaltyTest.cpp
        test/locomotion/SwingTrajectoryPlannerTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_locomotion ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_locomotion PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_locomotion ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_locomotion PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_trajectory_planners
        test/trajectory_planners/BaseTrajectoryPlannerTest.cpp  
        test/trajectory_planners/JointTrajectoryPlannerTest.cpp   
        test/trajectory_planners/ContactForceWrenchTrajectoryPlannerTest.cpp   
  )
  ament_target_dependencies(${PROJECT_NAME}_test_trajectory_planners ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_trajectory_planners PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_trajectory_planners ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_trajectory_planners PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_weight_compensation
        test/weight_compensation/PinocchioWeightCompensatorTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_weight_compensation ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_weight_compensation PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_weight_compensation ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_weight_compensation PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_torque_approx
        test/torque_approx/PinocchioTorqueApproximationCppAdTest.cpp     
  )
  ament_target_dependencies(${PROJECT_NAME}_test_torque_approx ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_torque_approx PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_torque_approx ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_torque_approx PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_kinematicsCppAD
        test/kinematics/PinocchioForwardEndEffectorKinematicsCppAdTest.cpp     
  )
  ament_target_dependencies(${PROJECT_NAME}_test_kinematicsCppAD ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_kinematicsCppAD PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_kinematicsCppAD ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_kinematicsCppAD PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_collision
        test/collision/CollisionSettingsTest.cpp
        test/collision/PinocchioForwardCollisionKinematicsCppAdTest.cpp     
        test/collision/PinocchoCollisionInterfaceTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_collision ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_collision PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_collision ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_collision PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_reference_manager
        test/reference_manager/LeggedReferenceManagerTest.cpp     
  )
  ament_target_dependencies(${PROJECT_NAME}_test_reference_manager ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_reference_manager PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_reference_manager ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_reference_manager PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_precomputation
          test/precomputation/LeggedPrecomputationTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_precomputation ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_precomputation PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_precomputation ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_precomputation PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_cost
        test/cost/TrajectoryTrackingCostTest.cpp
        test/cost/JointTorqueCostTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_cost ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_cost PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_cost ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_cost PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_soft_constraint
        test/soft_constraint/WrenchFrictionConeSoftConstraintTest.cpp
        test/soft_constraint/ForceFrictionConeSoftConstraintTest.cpp
        test/soft_constraint/JointLimitsSoftConstraintTest.cpp
        test/soft_constraint/JointTorqueLimitsSoftConstraintTest.cpp
        test/soft_constraint/SelfCollisionAvoidanceSoftConstraintTest.cpp
        test/soft_constraint/TerrainAvoidanceSoftConstraintTest.cpp
        test/soft_constraint/EndEffectorPlacementSoftConstraintTest.cpp
        test/soft_constraint/NormalOrientationSoftConstraintTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_soft_constraint ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_soft_constraint PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_soft_constraint ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_soft_constraint PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_interface
        test/robot_interface/LeggedInterfaceTest.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_interface ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_interface PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_interface ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_interface PUBLIC ${FLAGS})

  ################
  ## Benchmarks ##
  ################
#   find_package(ament_cmake_google_benchmark REQUIRED)

  # ament_add_google_benchmark(${PROJECT_NAME}_benchmark_locomotion 
  #       benchmarks/GaitDynamicPhaseControllerBenchmark.cpp
  #       benchmarks/SwingTrajectoryPlannerBenchmark.cpp
  # )
  # target_include_directories(${PROJECT_NAME}_benchmark_locomotion PRIVATE benchmarks/include)
  # target_link_libraries(${PROJECT_NAME}_benchmark_locomotion ${PROJECT_NAME})
  # target_compile_options(${PROJECT_NAME}_benchmark_locomotion PUBLIC ${FLAGS})

  # ament_add_google_benchmark(${PROJECT_NAME}_benchmark_trajectory_planners 
  #       benchmarks/BaseTrajectoryPlannerBenchmark.cpp
  #       benchmarks/JointTrajectoryPlannerBenchmark.cpp
  # )
  # target_include_directories(${PROJECT_NAME}_benchmark_trajectory_planners PRIVATE benchmarks/include)
  # target_link_libraries(${PROJECT_NAME}_benchmark_trajectory_planners ${PROJECT_NAME})
  # target_compile_options(${PROJECT_NAME}_benchmark_trajectory_planners PUBLIC ${FLAGS})

  # ament_add_google_benchmark(${PROJECT_NAME}_benchmark_reference
  #       benchmarks/LeggedReferenceManagerBenchmark.cpp
  # )
  # target_include_directories(${PROJECT_NAME}_benchmark_reference PRIVATE benchmarks/include)
  # target_link_libraries(${PROJECT_NAME}_benchmark_reference ${PROJECT_NAME})
  # target_compile_options(${PROJECT_NAME}_benchmark_reference PUBLIC ${FLAGS})

#   ament_add_google_benchmark(${PROJECT_NAME}_benchmark_weight_compensation
#         benchmarks/UtilsVsWrenchCompensator.cpp
#   )
#   target_include_directories(${PROJECT_NAME}_benchmark_weight_compensation PRIVATE benchmarks/include)
#   target_link_libraries(${PROJECT_NAME}_benchmark_weight_compensation ${PROJECT_NAME})
#   target_compile_options(${PROJECT_NAME}_benchmark_weight_compensation PUBLIC ${FLAGS})

endif()

ament_package()
