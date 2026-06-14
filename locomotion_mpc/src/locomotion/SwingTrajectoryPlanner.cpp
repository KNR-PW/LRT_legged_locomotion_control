//
// Created by rgrandia on 13.03.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 02.09.2025 
//

#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.hpp>


#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;
    using namespace terrain_model;
    using namespace floating_base_model;

    SwingTrajectoryPlanner::SwingTrajectoryPlanner(FloatingBaseModelInfo info,
      StaticSettings staticSettings,
      DynamicSettings initDynamicSettings,
      const forwardKinematics &forwardKinematics,
      const OverExtensionPenalty& overExtensionPenalty): modelInfo_(std::move(info)),
        staticSettings_(std::move(staticSettings)), 
        dynamicSettings_(std::move(initDynamicSettings)),
        forwardKinematics_(forwardKinematics),
        overExtensionPenalty_(overExtensionPenalty)
    {
      lastContacts_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
      feetNormalTrajectories_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
      feetNormalTrajectoriesEvents_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);

      nominalFootholdsPerLeg_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
      heuristicFootholdsPerLeg_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
    }

    void SwingTrajectoryPlanner::updateInvertedPendulumHeight(
      scalar_t invertedPendulumHeight)
    {
      invertedPendulumHeight_ = invertedPendulumHeight;
    }

    void SwingTrajectoryPlanner::updateTerrain(const terrain_model::TerrainModel& terrainModel) 
    {
      terrainModel_ = &terrainModel;
    }
    
    void SwingTrajectoryPlanner::updateDynamicSettings(const DynamicSettings& newDynamicSettings)
    {
      dynamicSettings_ = newDynamicSettings;
    }

    const SignedDistanceField* SwingTrajectoryPlanner::getSignedDistanceField() const 
    {
      if(terrainModel_) 
      {
        return terrainModel_->getSignedDistanceField();
      } 
      else 
      {
        return nullptr;
      }
    }

    void SwingTrajectoryPlanner::updateSwingMotions(scalar_t initTime, scalar_t finalTime,
     const ocs2::SystemObservation& currentObservation, const TargetTrajectories& targetTrajectories,
      const ModeSchedule& modeSchedule) 
    {
      if(!terrainModel_) 
      {
        throw std::runtime_error("[SwingTrajectoryPlanner] terrain cannot be null. " 
          "Update the terrain before planning swing motions");
      }

      if(invertedPendulumHeight_ <= 0.0)
      {
        throw std::runtime_error("[SwingTrajectoryPlanner] Inverted pendulum height less than 0.0. Please use updateInvertedPendulumHeight() before running swing trajectory planner!");
      }

      modeSchedule_ = modeSchedule;
      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;

      const std::vector<std::vector<ContactTiming>> contactTimingsPerLeg =
        extractContactTimingsPerLeg(modeSchedule_, numEndEffectors);

      const auto& currentState = currentObservation.state;
      const auto feetPositions = forwardKinematics_.getPosition(currentState);

      for (int i = 0; i < numEndEffectors; ++i) 
      {
        const auto &contactTimings = contactTimingsPerLeg[i];

        // Update last contacts
        if(!contactTimings.empty()) 
        {
          scalar_t expectedLiftOffTime;
          if(startsWithStancePhase(contactTimings)) 
          {
            // If currently in contact -> update expected liftoff.
            if(hasEndTime(contactTimings.front())) 
            {
              expectedLiftOffTime = contactTimings.front().end;
              updateLastContact(i, expectedLiftOffTime, feetPositions[i], *terrainModel_);
            } 
            else 
            {
              // Expected liftoff unknown, set to end of horizon
              expectedLiftOffTime = finalTime;
              updateLastContact(i, expectedLiftOffTime, feetPositions[i], *terrainModel_);
            }
          } 
          else 
          {
            /**
             * If currently in swing -> verify that liftoff was before the horizon.
             * If not, assume liftoff happened exactly at initTime 
             */ 
            if(lastContacts_[i].first > initTime) 
            {
              expectedLiftOffTime = initTime;
              updateLastContact(i, expectedLiftOffTime, feetPositions[i], *terrainModel_);
            }
          }
        }

        // Select heuristic footholds.
        heuristicFootholdsPerLeg_[i] = selectHeuristicFootholds(i, contactTimings, 
          targetTrajectories, initTime,
          currentObservation, finalTime);

        // Select terrain constraints based on the heuristic footholds.
        nominalFootholdsPerLeg_[i] = selectNominalFootholdTerrain(
          i, contactTimings, heuristicFootholdsPerLeg_[i], targetTrajectories,
          initTime, currentObservation, finalTime, *terrainModel_);

        // Create swing trajectories
        std::tie(feetNormalTrajectoriesEvents_[i], feetNormalTrajectories_[i]) =
          generateSwingTrajectories(i, contactTimings, finalTime);
      }
    }

    const FootPhase &SwingTrajectoryPlanner::getFootPhase(size_t endEffectorIndex, 
      scalar_t time) const 
    {
      const auto timeIndex = utils::findIndexInTimeArray(
        feetNormalTrajectoriesEvents_[endEffectorIndex], time);
      return *feetNormalTrajectories_[endEffectorIndex][timeIndex];
    }

    std::vector<vector3_t> SwingTrajectoryPlanner::getEndEffectorPositions(scalar_t time) const
    {
      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;
      std::vector<vector3_t> positions(numEndEffectors);

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        const auto& footPhase = getFootPhase(i, time);
        positions[i] = footPhase.getPositionInWorld(time);
      }
      return positions;
    }

    std::vector<vector3_t> SwingTrajectoryPlanner::getEndEffectorVelocities(scalar_t time) const
    {
      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;
      std::vector<vector3_t> velocities(numEndEffectors);

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        const auto& footPhase = getFootPhase(i, time);
        velocities[i] = footPhase.getVelocityInWorld(time);
      }
      return velocities;
    }

    std::vector<scalar_t> SwingTrajectoryPlanner::getEndEffectorClearances(
      scalar_t time) const
    {
      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;
      std::vector<scalar_t> footClearances(numEndEffectors);

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        const auto& footPhase = getFootPhase(i, time);
        footClearances[i] = footPhase.getMinimumFootClearance(time);
      }
      return footClearances;
    }

    std::vector<vector3_t> SwingTrajectoryPlanner::getEndEffectorSurfaceNormals(
      scalar_t time) const
    {
      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;
      std::vector<vector3_t> normals(numEndEffectors);

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        const auto& footPhase = getFootPhase(i, time);
        normals[i] = footPhase.normalDirectionInWorldFrame(time);
      }
      return normals;
    }

    using position_trajectories = std::vector<std::vector<vector3_t>>;
    position_trajectories SwingTrajectoryPlanner::getEndEffectorPositionTrajectories(
      std::vector<scalar_t> times) const
    {
      position_trajectories positions;
      positions.reserve(times.size());

      for(const auto time: times)
      {
        auto position = getEndEffectorPositions(time);
        positions.emplace_back(std::move(position));
      }
      return positions;
    }

    using velocity_trajectories = std::vector<std::vector<vector3_t>>;
      velocity_trajectories SwingTrajectoryPlanner::getEndEffectorVelocityTrajectories(
      std::vector<scalar_t> times) const
    {
      velocity_trajectories velocities;
      velocities.reserve(times.size());

      for(const auto time: times)
      {
        auto velocity = getEndEffectorVelocities(time);
        velocities.emplace_back(std::move(velocity));
      }
      return velocities;
    }

    using foot_clearance_trajectory = std::vector<std::vector<scalar_t>>;
    foot_clearance_trajectory SwingTrajectoryPlanner::getEndEffectorClearanceTrajectories(
      std::vector<scalar_t> times) const
    {
      foot_clearance_trajectory footClerances;
      footClerances.reserve(times.size());
      for(const auto time: times)
      {
        auto footClerance = getEndEffectorClearances(time);
        footClerances.emplace_back(std::move(footClerance));
      }
      return footClerances;
    }

    using normal_trajectories = std::vector<std::vector<vector3_t>>;
    normal_trajectories SwingTrajectoryPlanner::getEndEffectorNormalTrajectories(
      std::vector<scalar_t> times) const
    {
      normal_trajectories normals;
      normals.reserve(times.size());
      for(const auto time: times)
      {
        auto normal = getEndEffectorSurfaceNormals(time);
        normals.emplace_back(std::move(normal));
      }
      return normals;
    }

    SwingTrajectoryPlanner::EndEffectorTrajectoriesPoint SwingTrajectoryPlanner::getEndEffectorTrajectoryPoint(
      scalar_t time) const
    {
      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;

      EndEffectorTrajectoriesPoint point;
      point.positions.reserve(numEndEffectors);
      point.velocities.reserve(numEndEffectors);
      point.clearances.reserve(numEndEffectors);
      point.surfaceNormals.reserve(numEndEffectors);

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        const auto& footPhase = getFootPhase(i, time);
        point.positions.emplace_back(footPhase.getPositionInWorld(time));
        point.velocities.emplace_back(footPhase.getVelocityInWorld(time));
        point.clearances.emplace_back(footPhase.getMinimumFootClearance(time));
        point.surfaceNormals.emplace_back(footPhase.normalDirectionInWorldFrame(time));
      }
      
      return point;
    }

    SwingTrajectoryPlanner::EndEffectorTrajectories SwingTrajectoryPlanner::getEndEffectorTrajectories(
      std::vector<scalar_t> times) const
    {
      EndEffectorTrajectories trajectories;
      trajectories.positions.reserve(times.size());
      trajectories.velocities.reserve(times.size());
      trajectories.clearances.reserve(times.size());
      trajectories.surfaceNormals.reserve(times.size());

      for(const auto time: times)
      {
        auto point = getEndEffectorTrajectoryPoint(time);
        trajectories.positions.emplace_back(std::move(point.positions));
        trajectories.velocities.emplace_back(std::move(point.velocities));
        trajectories.clearances.emplace_back(std::move(point.clearances));
        trajectories.surfaceNormals.emplace_back(std::move(point.surfaceNormals));
      }

      return trajectories;
    }

    using FootPhasesStamped =  std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>>; 
    FootPhasesStamped SwingTrajectoryPlanner::generateSwingTrajectories(
      size_t endEffectorIndex, const std::vector<ContactTiming> &contactTimings, 
      scalar_t finalTime) const
    {
      std::vector<scalar_t> eventTimes;
      std::vector<std::unique_ptr<FootPhase>> footPhases;

      // First swing phase
      if(startsWithSwingPhase(contactTimings)) 
      {
        SwingPhase::SwingEvent liftOff{lastContacts_[endEffectorIndex].first, 
          staticSettings_.liftOffVelocity, &lastContacts_[endEffectorIndex].second};
        SwingPhase::SwingEvent touchDown = [&] 
        {
          if(touchesDownAtLeastOnce(contactTimings)) 
          {
            return SwingPhase::SwingEvent{contactTimings.front().start,
               staticSettings_.touchDownVelocity, 
               &nominalFootholdsPerLeg_[endEffectorIndex].front().getTerrainPlane()};
          } 
          else 
          {
            return SwingPhase::SwingEvent{finalTime + staticSettings_.referenceExtensionAfterHorizon,
              0.0, nullptr};
          }}();

        SwingPhase::SwingProfile swingProfile = getDynamicSwingProfile(endEffectorIndex);
        applySwingMotionScaling(liftOff, touchDown, swingProfile);

        footPhases.emplace_back(new SwingPhase(liftOff, touchDown, 
          swingProfile, terrainModel_));
      }

      // Loop through contact phases
      for (int i = 0; i < contactTimings.size(); ++i) 
      {
        const auto &currentContactTiming = contactTimings[i];
        const ConvexTerrain &nominalFoothold = nominalFootholdsPerLeg_[endEffectorIndex][i];
        
        // If phase starts after the horizon, we don't need to plan for it
        if(currentContactTiming.start > finalTime) 
        {
          break;
        }

        // generate contact phase
        if(hasStartTime(currentContactTiming)) 
        {
          eventTimes.push_back(currentContactTiming.start);
        }
        footPhases.emplace_back(new StancePhase(nominalFoothold, staticSettings_.terrainMargin));
        
        // If contact phase extends beyond the horizon, we can stop planning.
        if(!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) 
        {
          break;
        }

        // generate swing phase afterwards
        SwingPhase::SwingEvent liftOff{currentContactTiming.end, 
          staticSettings_.liftOffVelocity, &nominalFoothold.getTerrainPlane()};
        SwingPhase::SwingEvent touchDown = [&] 
        {
          const bool nextContactExists = (i + 1) < contactTimings.size();
          if(nextContactExists) 
          {
            return SwingPhase::SwingEvent{contactTimings[i + 1].start, 
              staticSettings_.touchDownVelocity, 
              &nominalFootholdsPerLeg_[endEffectorIndex][i + 1].getTerrainPlane()};
          } 
          else 
          {
            return SwingPhase::SwingEvent{finalTime + 
              staticSettings_.referenceExtensionAfterHorizon, 0.0, nullptr};
          }}();
        SwingPhase::SwingProfile swingProfile = getDynamicSwingProfile(endEffectorIndex);
        applySwingMotionScaling(liftOff, touchDown, swingProfile);
        eventTimes.push_back(currentContactTiming.end);
        footPhases.emplace_back(new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_));
      }
      return std::make_pair(eventTimes, std::move(footPhases));
    }

    void SwingTrajectoryPlanner::applySwingMotionScaling(SwingPhase::SwingEvent &liftOff, 
      SwingPhase::SwingEvent &touchDown, 
      SwingPhase::SwingProfile &swingProfile) const 
    {
      const scalar_t scaling = [&]() 
      {
        if(std::isnan(liftOff.time) || std::isnan(touchDown.time)) 
        {
          return 1.0;
        } 
        else 
        {
          return std::min(1.0, (touchDown.time - liftOff.time) / staticSettings_.swingTimeScale);
        }
      }();

      if(scaling < 1.0) 
      {
        liftOff.velocity *= scaling;
        touchDown.velocity *= scaling;
        swingProfile.sdfMidswingMargin = scaling * staticSettings_.sdfMidswingMargin;
        for (auto &node: swingProfile.nodes) 
        {
          node.swingHeight *= scaling;
          node.normalVelocity *= scaling;
        }
      }
    }

    std::vector<vector3_t> SwingTrajectoryPlanner::selectHeuristicFootholds(
      size_t endEffectorIndex, const std::vector<ContactTiming> &contactTimings,
      const TargetTrajectories &targetTrajectories, scalar_t initTime,
      const SystemObservation& currentObservation, scalar_t finalTime) const
    {
      // Zmp preparation : measured state
      const auto& currentState = currentObservation.state;
      const vector3_t initBaseOrientation = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(currentState, modelInfo_);
      const vector3_t initBaseLinearVelocityInBase = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(currentState, modelInfo_);
      const matrix3_t initBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        initBaseOrientation);
      const vector3_t initBaseLinearVelocityInWorld = initBaseRotationMatrix 
        * initBaseLinearVelocityInBase;

      // Zmp preparation : desired state
      const vector_t initDesiredState = targetTrajectories.getDesiredState(initTime);
      const vector3_t initBaseDesiredOrientation = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(initDesiredState, modelInfo_);
      const vector3_t initBaseDesiredLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(initDesiredState, modelInfo_);
      const matrix3_t initBaseDesiredRotationMatrix = getRotationMatrixFromZyxEulerAngles(
      initBaseDesiredOrientation);
      const vector3_t initDesiredBaseLinearVelocityInWorld = initBaseDesiredRotationMatrix 
        * initBaseDesiredLinearVelocity;

      /** 
       * Compute zmp / inverted pendulum foot placement offset: 
       * delta p = sqrt(h / g) * (v - v_des) 
       */ 
      scalar_t pendulumFrequency = std::sqrt(invertedPendulumHeight_ / PLUS_GRAVITY_VALUE);
      scalar_t zmpX = pendulumFrequency * (initBaseLinearVelocityInWorld.x() 
        - initDesiredBaseLinearVelocityInWorld.x());
      scalar_t zmpY = pendulumFrequency * (initBaseLinearVelocityInWorld.y() 
        - initDesiredBaseLinearVelocityInWorld.y());
      const vector3_t zmpReactiveOffset = {zmpX, zmpY, 0.0};

      // Heuristic footholds to fill
      std::vector<vector3_t> heuristicFootholds;

      // Heuristic foothold is equal to current foothold for legs in contact
      if(startsWithStancePhase(contactTimings)) 
      {
        heuristicFootholds.push_back(lastContacts_[endEffectorIndex].second.getPosition());
      }

      // For future contact phases, use TargetTrajectories at halve the contact phase
      size_t contactCount = 0;
      for (const auto &contactPhase: contactTimings) 
      {
        if(hasStartTime(contactPhase)) 
        {
          const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
          const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

          // Compute foot position from cost desired trajectory
          vector_t state = targetTrajectories.getDesiredState(middleContactTime);

          /**
           * IMPORTANT: URDF NEEDS TO HAVE STRAIGHT LEGS WITH ZERO JOINT POSITIONS 
           * IN ORDER TO GET POSITIONS BELOW HIP HERE!!!!
           * ALSO JOINT POSITIONS IN THIS TARGET TRAJECTORIES NEEDS TO BE ZERO!!!
           */
          access_helper_functions::getJointPositions(state, modelInfo_).setZero();
          vector3_t referenceFootholdPositionInWorld = forwardKinematics_.getPosition(state)[endEffectorIndex];

          // Add ZMP offset to the first upcoming foothold.
          if(contactCount == 0) 
          {
            referenceFootholdPositionInWorld += zmpReactiveOffset;
          }

          // One foothold added per contactPhase
          heuristicFootholds.push_back(referenceFootholdPositionInWorld);

          /**
           * Can stop for this end effector if we have processed one contact
           * phase after (or extending across) the horizon
           */ 
          if(contactEndTime > finalTime) 
          {
            break;
          }
        }
          ++contactCount;
      }

      return heuristicFootholds;
    }

    std::vector<ConvexTerrain> SwingTrajectoryPlanner::selectNominalFootholdTerrain(
      size_t endEffectorIndex, const std::vector<ContactTiming> &contactTimings, 
      const std::vector<vector3_t> &heuristicFootholds, 
      const TargetTrajectories &targetTrajectories, scalar_t initTime, 
      const SystemObservation& currentObservation, scalar_t finalTime, 
      const TerrainModel &terrainModel) const 
    {
      // Will increment the heuristic each time after selecting a nominalFootholdTerrain
      auto heuristicFootholdIt = heuristicFootholds.cbegin();
      std::vector<ConvexTerrain> nominalFootholdTerrain;

      // Nominal foothold is equal to current foothold for end effectors in contact
      if(startsWithStancePhase(contactTimings)) 
      {
        std::vector<vector2_t> emptyBoundry;
        const auto plane = lastContacts_[endEffectorIndex].second;
        ConvexTerrain convexTerrain(plane, emptyBoundry);
        nominalFootholdTerrain.push_back(convexTerrain);
        ++heuristicFootholdIt; // Skip this heuristic. Use lastContact directly
      }

      // For future contact phases
      for (const auto &contactPhase: contactTimings) 
      {
        if(hasStartTime(contactPhase)) 
        {
          const scalar_t timeTillContact = contactPhase.start - initTime;
          const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
          const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

          // Get previous foothold if there was one at this time
          const FootPhase *previousIterationContact = getFootPhaseIfInContact(endEffectorIndex, 
            middleContactTime);

          if(timeTillContact < staticSettings_.previousFootholdTimeDeadzone 
            && previousIterationContact != nullptr) 
          {
            // Simply copy the information out of the previous iteration
            nominalFootholdTerrain.push_back(*previousIterationContact->nominalFootholdConstraint());
            ++heuristicFootholdIt; // Skip this heuristic. Using the previous terrain instead
          } 
          else 
          {
            // Select the terrain base on the heuristic
            vector3_t referenceFootholdPositionInWorld = *heuristicFootholdIt;

            // Filter w.r.t. previous foothold
            if(previousIterationContact != nullptr) 
            {
              referenceFootholdPositionInWorld = filterFoothold(
                referenceFootholdPositionInWorld, 
                previousIterationContact->nominalFootholdLocation());
            }

            if(contactPhase.start < finalTime) 
            {
              const vector_t desiredState = targetTrajectories.getDesiredState(contactPhase.start);
              auto penaltyFunction = overExtensionPenalty_.getPenalty(endEffectorIndex, desiredState);
              ConvexTerrain convexTerrain = terrainModel.getConvexTerrainAtPositionInWorld(
                  referenceFootholdPositionInWorld, penaltyFunction);
              nominalFootholdTerrain.push_back(convexTerrain);
              ++heuristicFootholdIt;
            } 
            else 
            {
              // After the horizon -> we are only interested in the position and orientation
              std::vector<vector2_t> emptyBoundry;
              const auto plane = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(
                referenceFootholdPositionInWorld);
              ConvexTerrain convexTerrain(plane, emptyBoundry);
              nominalFootholdTerrain.push_back(convexTerrain);
              ++heuristicFootholdIt;
            }
          }

          /** 
           * Can stop for this end effector if we have processed one contact 
           * phase after (or extending across) the horizon
           */
          if(contactEndTime > finalTime) 
          {
            break;
          }
        }
      }

      return nominalFootholdTerrain;
    }

    void SwingTrajectoryPlanner::updateLastContact(size_t endEffectorIndex, scalar_t expectedLiftOff,
      const vector3_t &currentFootPosition,
      const TerrainModel &terrainModel) 
    {
      // Get orientation from terrain model, position from the kinematics
      const auto lastContactTerrain = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(currentFootPosition);
      TerrainPlane newPlane(currentFootPosition, lastContactTerrain.getOrientationToTerrain());
      lastContacts_[endEffectorIndex] = {expectedLiftOff, newPlane};
    }

    SwingPhase::SwingProfile SwingTrajectoryPlanner::getDynamicSwingProfile(
      size_t endEffectorIndex) const 
    {
      SwingPhase::SwingProfile swingProfile;
      swingProfile.sdfMidswingMargin = staticSettings_.sdfMidswingMargin;
      swingProfile.maxSwingHeightAdaptation = staticSettings_.maxSwingHeightAdaptation;

      SwingPhase::SwingProfile::Node midPoint;
      midPoint.phase = dynamicSettings_.phases[endEffectorIndex];
      midPoint.swingHeight = dynamicSettings_.swingHeights[endEffectorIndex];
      midPoint.normalVelocity = 0.0;
      midPoint.tangentialProgress = dynamicSettings_.tangentialProgresses[endEffectorIndex];
      midPoint.tangentialVelocityFactor = dynamicSettings_.tangentialVelocityFactors[endEffectorIndex];
      swingProfile.nodes.push_back(midPoint);
      return swingProfile;
    }

    scalar_t SwingTrajectoryPlanner::getContactEndTime(const ContactTiming &contactPhase, 
      scalar_t finalTime) const 
    {
      return hasEndTime(contactPhase) ? contactPhase.end 
        : std::max(finalTime + staticSettings_.referenceExtensionAfterHorizon, contactPhase.start);
    }

    const FootPhase *SwingTrajectoryPlanner::getFootPhaseIfInContact(size_t endEffectorIndex, 
      scalar_t time) const 
    {
      const FootPhase *previousIterationContact = nullptr;
      if(!feetNormalTrajectories_[endEffectorIndex].empty()) 
      {
        const auto &footPhase = getFootPhase(endEffectorIndex, time);
        if(footPhase.contactFlag()) 
        {
          previousIterationContact = &footPhase;
        }
      }
      return previousIterationContact;
    }

    vector3_t SwingTrajectoryPlanner::filterFoothold(const vector3_t &newFoothold,
      const vector3_t &previousFoothold) const 
    {
      // Apply Position deadzone and low pass filter
      if((newFoothold - previousFoothold).norm() < staticSettings_.previousFootholdDeadzone) 
      {
        return previousFoothold;
      } 
      else 
      {
        // low pass filter
        const scalar_t lambda = staticSettings_.previousFootholdFactor;
        return lambda * previousFoothold + (1.0 - lambda) * newFoothold;
      }
    }

    SwingTrajectoryPlanner::FootTangentialConstraintTrajectories 
      SwingTrajectoryPlanner::getFootTangentialConstraintTrajectories()
    {
      const auto& eventTimes = modeSchedule_.eventTimes;
      const auto& modeSequence = modeSchedule_.modeSequence;

      const size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;
      const size_t trajectorySize = modeSequence.size();

      FootTangentialConstraintTrajectories trajectory;
      trajectory.times = modeSchedule_.eventTimes;
      trajectory.constraints.reserve(trajectorySize);

      // Dont run first mode (foot is already on the ground)
      std::vector<FootTangentialConstraintMatrix> firstConstraints(numEndEffectors, 
        FootTangentialConstraintMatrix());
      trajectory.constraints.push_back(std::move(firstConstraints));

      for(size_t i = 1; i < trajectorySize - 1; ++i)
      {
        const contact_flags_t contactFlags = modeNumber2ContactFlags(modeSequence[i]);
        std::vector<FootTangentialConstraintMatrix> newConstraints(numEndEffectors);

        for(size_t j = 0; j < numEndEffectors; ++j)
        {
          const scalar_t midTime = eventTimes[i] + 0.5 * (eventTimes[i + 1] - eventTimes[i]);
          const auto& footPhase = getFootPhase(j, eventTimes[i]);
          const FootTangentialConstraintMatrix* newConstraintPtr = footPhase.getFootTangentialConstraintInWorldFrame();
          if(contactFlags[j] && newConstraintPtr)
          {
            newConstraints[j] = *newConstraintPtr;
          }
          else
          {
            newConstraints[j] = FootTangentialConstraintMatrix();
          }
        }
        trajectory.constraints.push_back(std::move(newConstraints));
      }

      trajectory.constraints.push_back(trajectory.constraints.back());
      return trajectory;
    }

    std::vector<ConvexTerrain> SwingTrajectoryPlanner::getNominalFootholds(
      size_t endEffectorIndex) const 
    { 
      return nominalFootholdsPerLeg_[endEffectorIndex]; 
    }

    std::vector<vector3_t> SwingTrajectoryPlanner::getHeuristicFootholds(
      size_t endEffectorIndex) const 
    { 
      return heuristicFootholdsPerLeg_[endEffectorIndex]; 
    }
   
    const SwingTrajectoryPlanner::StaticSettings& SwingTrajectoryPlanner::getStaticSettings() const 
    { 
      return staticSettings_; 
    }
        
    const SwingTrajectoryPlanner::DynamicSettings& SwingTrajectoryPlanner::getDynamicSettings() const 
    { 
      return dynamicSettings_; 
    }

    SwingTrajectoryPlanner::StaticSettings loadSwingPlannerStaticSettings(
      const std::string& filename, const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Swing Planner Static Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      SwingTrajectoryPlanner::StaticSettings settings;

      loadData::loadPtreeValue(pt, settings.liftOffVelocity, 
        fieldName + ".liftOffVelocity", verbose);
      if(settings.liftOffVelocity < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Liftoff velocity smaller than 0!");
      }

      loadData::loadPtreeValue(pt, settings.touchDownVelocity, 
        fieldName + ".touchDownVelocity", verbose);
      if(settings.touchDownVelocity > 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Touchdown velocity bigger than 0!");
      }

      loadData::loadPtreeValue(pt, settings.errorGain, 
        fieldName + ".errorGain", verbose);
      if(settings.errorGain < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Error gain less than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.swingTimeScale, 
        fieldName + ".swingTimeScale", verbose);
      if(settings.swingTimeScale < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Swing time scale smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.sdfMidswingMargin, 
        fieldName + ".sdfMidswingMargin", verbose);

      loadData::loadPtreeValue(pt, settings.terrainMargin, 
        fieldName + ".terrainMargin", verbose);

      loadData::loadPtreeValue(pt, settings.previousFootholdFactor, 
        fieldName + ".previousFootholdFactor", verbose);
       if(settings.previousFootholdFactor < 0.0 || settings.previousFootholdFactor > 1.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Previous foothold factor needs to be between [0.0, 1.0]!");
      }

      loadData::loadPtreeValue(pt, settings.previousFootholdDeadzone, 
        fieldName + ".previousFootholdDeadzone", verbose);
       if(settings.previousFootholdDeadzone < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Previous foothold deadzone smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.previousFootholdTimeDeadzone, 
        fieldName + ".previousFootholdTimeDeadzone", verbose);
       if(settings.previousFootholdTimeDeadzone < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Previous foothold time deadzone smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.nominalLegExtension, 
        fieldName + ".nominalLegExtension", verbose);
       if(settings.nominalLegExtension < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Nominal leg extension smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.legOverExtensionWeight, 
        fieldName + ".legOverExtensionWeight", verbose);
       if(settings.legOverExtensionWeight < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Leg over extension penalty weight smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.referenceExtensionAfterHorizon, 
        fieldName + ".referenceExtensionAfterHorizon", verbose);
       if(settings.referenceExtensionAfterHorizon < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Reference extension after horizon smaller than 0.0!");
      }

      loadData::loadPtreeValue(pt, settings.maxSwingHeightAdaptation, 
        fieldName + ".maxSwingHeightAdaptation", verbose);
       if(settings.maxSwingHeightAdaptation < 0.0)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Max swing height adaptation smaller than 0.0!");
      }

      if(verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return settings;
    }

    SwingTrajectoryPlanner::DynamicSettings loadSwingPlannerDynamicSettings(
      const std::string& filename, const FloatingBaseModelInfo& info,
      const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      SwingTrajectoryPlanner::DynamicSettings settings;

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Swing Planner Static Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      const size_t endEffectorNum = info.numThreeDofContacts + info.numSixDofContacts;

      loadData::loadStdVector(filename, fieldName + ".swingHeights", settings.swingHeights, verbose);
      if(settings.swingHeights.size() != endEffectorNum)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Swing heights have wrong size!");
      }
      for(const auto& swingHeight: settings.swingHeights)
      {
        if(swingHeight < 0.0)
        {
          throw std::invalid_argument("[SwingTrajectoryPlanner]: Swing height smaller than 0.0!");
        }
      }

      loadData::loadStdVector(filename, fieldName + ".phases", settings.phases, verbose);
      if(settings.phases.size() != endEffectorNum)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Phases have wrong size!");
      }
      for(const auto& phase: settings.phases)
      {
        if(phase < 0.0 || phase > 1.0)
        {
          throw std::invalid_argument("[SwingTrajectoryPlanner]: Phase needs to be between [0.0, 1.0)!");
        }
      }

      loadData::loadStdVector(filename, fieldName + ".tangentialProgresses", settings.tangentialProgresses, verbose);
      if(settings.tangentialProgresses.size() != endEffectorNum)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Tangential progresses have wrong size!");
      }
      for(const auto& tangentialProgress: settings.tangentialProgresses)
      {
        if(tangentialProgress < 0.0 || tangentialProgress > 1.0)
        {
          throw std::invalid_argument("[SwingTrajectoryPlanner]: Tangential progress needs to be between [0.0, 1.0]!");
        }
      }

      loadData::loadStdVector(filename, fieldName + ".tangentialVelocityFactors", settings.tangentialVelocityFactors, verbose);
      if(settings.tangentialVelocityFactors.size() != endEffectorNum)
      {
        throw std::invalid_argument("[SwingTrajectoryPlanner]: Tangential velocity factors have wrong size!");
      }
      for(const auto& tangentialVelocityFactor: settings.tangentialVelocityFactors)
      {
        if(tangentialVelocityFactor < 0.0)
        {
          throw std::invalid_argument("[SwingTrajectoryPlanner]: Tangential velocity factor smaller than 0.0!");
        }
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return settings;
    }
  } // namespace locomotion
} // namespace legged_locomotion_mpc
  