
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>

#include <algorithm>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {

    using namespace ocs2;
    using namespace terrain_model;
    using namespace floating_base_model;
    using namespace quaterion_euler_transforms;

    void unwrapYawAngle(TargetTrajectories& targetTrajectory, 
      const FloatingBaseModelInfo& modelInfo)
    {
      for(size_t i = 1; i < targetTrajectory.stateTrajectory.size(); ++i)
      {
        const auto& previousState = targetTrajectory.stateTrajectory[i - 1];
        const auto previousEulerAngles = access_helper_functions::getBaseOrientationZyx(
          previousState, modelInfo);

        auto& newState = targetTrajectory.stateTrajectory[i];
        auto newEulerAngles = access_helper_functions::getBaseOrientationZyx(
          newState, modelInfo);

        const scalar_t yawDifference = newEulerAngles.x() - previousEulerAngles.x();

        static const scalar_t EulerYawJump = 2.0 * M_PI;

        if(yawDifference > M_PI)
        {
          newEulerAngles.x() -= EulerYawJump;
        }
        else if(yawDifference < -M_PI)
        {
          newEulerAngles.x() += EulerYawJump;
        }
      }
    }

    matrix3_t projectRotationMatrixOnPlane(const matrix3_t& rotationMatrixToWorld, const TerrainPlane& plane)
    {
      const vector3_t headingVector = rotationMatrixToWorld.col(0);
      const vector3_t xAxis = plane.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
      const vector3_t zAxis = plane.getSurfaceNormalInWorld();
      const vector3_t yAxis = zAxis.cross(xAxis);

      matrix3_t projectedRotationMatrix;
      projectedRotationMatrix.col(0) = xAxis;
      projectedRotationMatrix.col(1) = yAxis;
      projectedRotationMatrix.col(2) = zAxis;
        
      return projectedRotationMatrix;
    }

    vector2_t rotateVector2D(const vector2_t& vector, scalar_t angle)
    {
      vector2_t returnVector;
      const scalar_t sinValue = sin(angle);
      const scalar_t cosValue = cos(angle);
      returnVector.x() = vector.x() * cosValue - vector.y() * sinValue;
      returnVector.y() = vector.x() * sinValue + vector.y() * cosValue;
      return returnVector;
    }

    BaseTrajectoryPlanner::BaseTrajectoryPlanner(FloatingBaseModelInfo modelInfo,
      BaseTrajectoryPlanner::StaticSettings settings): 
        modelInfo_(std::move(modelInfo)), settings_(std::move(settings)) {}

    scalar_t BaseTrajectoryPlanner::getBaseHeight() const
    {
      return currentBaseHeight_;
    }
      
    void BaseTrajectoryPlanner::updateTerrain(const terrain_model::TerrainModel& terrainModel)
    {
      terrainModel_ = &terrainModel;
    }

    void BaseTrajectoryPlanner::updateTargetTrajectory(scalar_t initTime,
      scalar_t finalTime, const BaseTrajectoryPlanner::BaseReferenceCommand& command,
      const SystemObservation& currentObservation,
      TargetTrajectories& targetTrajectories)
    {
      if(!terrainModel_) 
      {
        throw std::runtime_error("[BaseTrajectoryPlanner] terrain cannot be null. " 
          "Update the terrain before planning base trajectory");
      }

      const size_t referenceSize = (finalTime - initTime) / settings_.deltaTime + 1;

      targetTrajectories.timeTrajectory.reserve(referenceSize);

      targetTrajectories.stateTrajectory.resize(referenceSize, 
        vector_t::Zero(modelInfo_.stateDim));

      targetTrajectories.inputTrajectory.resize(referenceSize, 
        vector_t::Zero(modelInfo_.inputDim));

      const auto clampedCommand = clampReferenceCommand(command);

      const vector3_t linearDelta{
        clampedCommand.baseHeadingVelocity * settings_.deltaTime, 
        clampedCommand.baseLateralVelocity * settings_.deltaTime, 
        clampedCommand.baseVerticalVelocity * settings_.deltaTime};

      const vector3_t angularDelta{0.0, 0.0, clampedCommand.yawRate * settings_.deltaTime};

      const pinocchio::Motion twistDelta(linearDelta, angularDelta);

      const pinocchio::SE3 SE3Delta = pinocchio::exp6(
        twistDelta);
      
      const auto& initialState = currentObservation.state;

      const vector3_t currentBasePosition = floating_base_model::
        access_helper_functions::getBasePosition(initialState, modelInfo_);

      const vector3_t currentBaseOrientationZyx = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(initialState, 
        modelInfo_);

      const matrix3_t currentBaseRotation = getRotationMatrixFromZyxEulerAngles(
        currentBaseOrientationZyx);

      const scalar_t invDeltaTime = 1 / settings_.deltaTime;

      pinocchio::SE3 currentTransform(currentBaseRotation, currentBasePosition);

      targetTrajectories.timeTrajectory.push_back(initTime);

      vector_t& initStateVector = targetTrajectories.stateTrajectory[0];

      access_helper_functions::
        getBasePosition(initStateVector, modelInfo_) = currentBasePosition;

      access_helper_functions::
        getBaseOrientationZyx(initStateVector, modelInfo_) = currentBaseOrientationZyx;
      
      // Check if base height is initialized
      if(!baseHeightInitialized_)
      {
        baseHeightInitialized_ = true;

        const auto currentPlane = 
          terrainModel_->getLocalTerrainAtPositionInWorldAlongGravity(currentBasePosition);

        currentBaseHeight_ = (currentBasePosition - currentPlane.getPosition()).dot(
          currentPlane.getSurfaceNormalInWorld());
        currentBaseHeight_ = std::clamp(currentBaseHeight_, settings_.minimumBaseHeight, 
          settings_.maximumBaseHeight);
      }

      for (size_t i = 1; i < referenceSize; ++i) 
      {
        // New base height normal to plane 
        currentBaseHeight_ = std::clamp(currentBaseHeight_ + 
          clampedCommand.baseVerticalVelocity * settings_.deltaTime, settings_.minimumBaseHeight, 
          settings_.maximumBaseHeight);
        
        // Get new position and orientation with current body velocities
        pinocchio::SE3 newTransform = currentTransform * SE3Delta;

        vector3_t newBasePosition = newTransform.translation();
        matrix3_t newBaseRotation = newTransform.rotation();

        vector3_t newBaseOrientationZyx = getEulerAnglesFromRotationMatrix(
          newBaseRotation);

        // Get new position and orientation on new plane
        const TerrainPlane newBaseTerrain = projectedHeadingPlane({newBasePosition.x(), 
          newBasePosition.y()}, newBaseOrientationZyx.x());

        newBaseRotation = projectRotationMatrixOnPlane(newBaseRotation, 
          newBaseTerrain);

        newBasePosition = newBaseTerrain.getPosition();

        // Add new height to position, taking into accoount fact that base is on terrain plane
        newBasePosition.z() += currentBaseHeight_ / 
          newBaseTerrain.getSurfaceNormalInWorld().z();
      
        newBaseOrientationZyx = getEulerAnglesFromRotationMatrix(
          newBaseRotation);
        
        // Make new transform from base and rotation on plane
        newTransform = pinocchio::SE3(newBaseRotation, newBasePosition);
        
        // Get twist from current -> new transforms
        const pinocchio::Motion newTwist = pinocchio::log6(
          currentTransform.actInv(newTransform)) * invDeltaTime;

        vector_t& currentStateVector = targetTrajectories.stateTrajectory[i];

        access_helper_functions::
          getBasePosition(currentStateVector, modelInfo_) = newBasePosition;

        access_helper_functions::
          getBaseOrientationZyx(currentStateVector, modelInfo_) = 
          newBaseOrientationZyx;

        vector_t& previousStateVector = targetTrajectories.stateTrajectory[i - 1];

        access_helper_functions::
          getBaseVelocity(previousStateVector, modelInfo_) = newTwist.toVector();

        targetTrajectories.timeTrajectory.push_back(
          targetTrajectories.timeTrajectory.back() + settings_.deltaTime);
        
        currentTransform = newTransform;
      }

      const vector_t& almostLastStateVector = targetTrajectories.stateTrajectory[referenceSize - 2];
      vector_t& lastStateVector = targetTrajectories.stateTrajectory[referenceSize - 1];

      access_helper_functions::
        getBaseVelocity(lastStateVector, modelInfo_) = 
        access_helper_functions::
        getBaseVelocity(almostLastStateVector, modelInfo_);

      unwrapYawAngle(targetTrajectories, modelInfo_);
    }

    const BaseTrajectoryPlanner::StaticSettings& BaseTrajectoryPlanner::getStaticSettings() const
    {
      return settings_;
    }

    TerrainPlane BaseTrajectoryPlanner::projectedHeadingPlane(
      vector2_t baseXYPositionInWorld, scalar_t yawOnPlane)
    {
      const vector2_t lfOffsetLocal(settings_.nominalBaseWidthHeading / 2.0, 
          settings_.nominalBaseWidthLateral / 2.0);
      const vector2_t rfOffsetLocal(settings_.nominalBaseWidthHeading / 2.0, 
        -settings_.nominalBaseWidthLateral / 2.0);
      const vector2_t lhOffsetLocal(-settings_.nominalBaseWidthHeading / 2.0, 
        settings_.nominalBaseWidthLateral / 2.0);
      const vector2_t rhOffsetLocal(-settings_.nominalBaseWidthHeading / 2.0, 
        -settings_.nominalBaseWidthLateral / 2.0);
        
      // Rotate from heading to world frame
      vector2_t lfOffsetGlobal = rotateVector2D(lfOffsetLocal, yawOnPlane);
      vector2_t rfOffsetGlobal = rotateVector2D(rfOffsetLocal, yawOnPlane);
      vector2_t lhOffsetGlobal = rotateVector2D(lhOffsetLocal, yawOnPlane);
      vector2_t rhOffsetGlobal = rotateVector2D(rhOffsetLocal, yawOnPlane);

      // shift by base center
      lfOffsetGlobal += baseXYPositionInWorld;
      rfOffsetGlobal += baseXYPositionInWorld;
      lhOffsetGlobal += baseXYPositionInWorld;
      rhOffsetGlobal += baseXYPositionInWorld;

      // Get position with smoothed terrain height
      const vector3_t lfVerticalProjection = terrainModel_->getSmoothedPositon(lfOffsetGlobal);
      const vector3_t rfVerticalProjection = terrainModel_->getSmoothedPositon(rfOffsetGlobal);
      const vector3_t lhVerticalProjection = terrainModel_->getSmoothedPositon(lhOffsetGlobal);
      const vector3_t rhVerticalProjection = terrainModel_->getSmoothedPositon(rhOffsetGlobal);

      const TerrainPlane basePlane = computeTerrainPlane({lfVerticalProjection, 
        rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});
        
      const vector3_t eulerAngles(yawOnPlane, 0.0, 0.0);
        
      const matrix3_t oldRotation = getRotationMatrixFromZyxEulerAngles(eulerAngles);

      const matrix3_t newRotationMatrix = projectRotationMatrixOnPlane(oldRotation, basePlane);

      return TerrainPlane(basePlane.getPosition(), newRotationMatrix.transpose());
    }

    BaseTrajectoryPlanner::BaseReferenceCommand BaseTrajectoryPlanner::clampReferenceCommand(
      const BaseReferenceCommand& command) const
    {
      BaseReferenceCommand clampedCommand;

      clampedCommand.baseHeadingVelocity = std::clamp(command.baseHeadingVelocity, 
        -settings_.maximumBaseHeadingVelocity, settings_.maximumBaseHeadingVelocity);

      clampedCommand.baseLateralVelocity = std::clamp(command.baseLateralVelocity, 
        -settings_.maximumBaseLateralVelocity, settings_.maximumBaseLateralVelocity);

      clampedCommand.baseVerticalVelocity = std::clamp(command.baseVerticalVelocity,  
        -settings_.maximumBaseVerticalVelocity, settings_.maximumBaseVerticalVelocity);

      clampedCommand.yawRate = std::clamp(command.yawRate, 
        -settings_.maximumYawRate, settings_.maximumYawRate);

      return clampedCommand;
    }

    BaseTrajectoryPlanner::StaticSettings loadBasePlannerStaticSettings(
      const std::string& filename, const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Base Planner Static Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      BaseTrajectoryPlanner::StaticSettings settings;

      loadData::loadPtreeValue(pt, settings.deltaTime, 
        fieldName + ".deltaTime", verbose);
      if(settings.deltaTime < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Delta time smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.minimumBaseHeight, 
        fieldName + ".minimumBaseHeight", verbose);
      if(settings.minimumBaseHeight < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Minimum base height smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.maximumBaseHeight, 
        fieldName + ".maximumBaseHeight", verbose);
      if(settings.maximumBaseHeight < 0.0 || settings.maximumBaseHeight < settings.minimumBaseHeight)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Maximum base height smaller than 0.0 or minimum base height!");
      }

      loadData::loadPtreeValue(pt, settings.maximumBaseHeadingVelocity, 
        fieldName + ".maximumBaseHeadingVelocity", verbose);
      if(settings.maximumBaseHeadingVelocity < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Maximum base heading velocity smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.maximumBaseLateralVelocity, 
        fieldName + ".maximumBaseLateralVelocity", verbose);
      if(settings.maximumBaseLateralVelocity < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Maximum base lateral velocity smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.maximumBaseVerticalVelocity, 
        fieldName + ".maximumBaseVerticalVelocity", verbose);
      if(settings.maximumBaseVerticalVelocity < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Maximum base vertical velocity smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.maximumYawRate, 
        fieldName + ".maximumYawRate", verbose);
      if(settings.maximumYawRate < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Maximum base yaw rate smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.nominalBaseWidthLateral, 
        fieldName + ".nominalBaseWidthLateral", verbose);
      if(settings.nominalBaseWidthLateral < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Nominal lateral base width smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.nominalBaseWidthHeading, 
        fieldName + ".nominalBaseWidthHeading", verbose);
      if(settings.nominalBaseWidthHeading < 0.0)
      {
        throw std::invalid_argument("[BaseTrajectoryPlanner]: Nominal heading base width smaller than 0.0!");
      }

      if(verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return settings;
    }
	} // namespace planners
} // namespace legged_locomotion_mpc

