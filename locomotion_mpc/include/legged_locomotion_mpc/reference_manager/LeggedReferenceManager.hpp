// Copyright (c) 2025, Bartłomiej Krajewski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __LEGGED_REFERENCE_MANAGER_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_REFERENCE_MANAGER_LEGGED_LOCOMOTION_MPC__

#include <future>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_core/thread_support/BufferedPointer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_mpc/SystemObservation.h>

#include <legged_locomotion_mpc/common/Types.hpp>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>

namespace legged_locomotion_mpc
{
  /**
  * Manages the ModeSchedule and the TargetTrajectories for switched model.
  */
  class LeggedReferenceManager: public ocs2::ReferenceManager
  {
    public:

    struct Settings
    {
      /**
       * If deltaTime > maximumReferenceSampleInterval, subsampled target trajectory 
       * and other references
       */
      ocs2::scalar_t maximumReferenceSampleInterval = 0.05;

      /**
       * Gait update offset for stability, so the MPC can prepare itself for changed 
       * gait. Should be more than MPC frequency and solver time steps minimum.
       */
      ocs2::scalar_t gaitUpdateOffset = 0.1;

      /**
       * Option for generating target trajectory in parallel to MPC
       * on another thread
       */
      bool threaded = false;
    };
  
    LeggedReferenceManager(floating_base_model::FloatingBaseModelInfo modelInfo,
      LeggedReferenceManager::Settings settings,
      locomotion::GaitPlanner&& gaitPlanner,
      locomotion::SwingTrajectoryPlanner&& swingPlanner,
      planners::BaseTrajectoryPlanner&& basePlanner,
      planners::JointTrajectoryPlanner&& jointPlanner,
      planners::ContactForceWrenchTrajectoryPlanner&& forcePlanner);

    ~LeggedReferenceManager() override;

    void initialize(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, 
      const ocs2::SystemObservation& currentObservation, 
      const contact_flags_t& currentContactFlags,
      const locomotion::GaitDynamicParameters& currentGaitParameters,
      const locomotion::SwingTrajectoryPlanner::DynamicSettings& currentSwingParameters,
      std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel);

    void updateCommand(
      const planners::BaseTrajectoryPlanner::BaseReferenceCommand& currentCommand);

    void updateContactFlags(const contact_flags_t& currentContactFlags);

    void updateGaitParemeters(const locomotion::GaitDynamicParameters& currentGaitParameters);

    void updateTerrainModel(
      std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel);

    void updateSwingParameters(
      const locomotion::SwingTrajectoryPlanner::DynamicSettings& currentSwingParameters);

    void preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, 
      const ocs2::vector_t& initState) override;

    const contact_flags_t getContactFlags(ocs2::scalar_t time) const;

    const terrain_model::TerrainModel& getTerrainModel() const;
    
    using EndEffectorTrajectoriesPoint = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::EndEffectorTrajectoriesPoint;
    EndEffectorTrajectoriesPoint getEndEffectorTrajectoryPoint(ocs2::scalar_t time) const;

    using FootTangentialConstraintMatrix = legged_locomotion_mpc::locomotion::FootTangentialConstraintMatrix;
    const std::vector<FootTangentialConstraintMatrix>& getEndEffectorConstraintMatrixes(
      ocs2::scalar_t time) const;
    
    private:

      void generateNewTargetTrajectories(ocs2::scalar_t initTime, ocs2::scalar_t finalTime);
    
      const floating_base_model::FloatingBaseModelInfo modelInfo_;
      const Settings settings_;

      std::future<void> newTrajectories_;

      ocs2::BufferedValue<contact_flags_t> currentContactFlags_;
      ocs2::BufferedValue<locomotion::GaitDynamicParameters> currentGaitParameters_;
      ocs2::BufferedValue<locomotion::SwingTrajectoryPlanner::DynamicSettings> currentSwingParameters_;
      ocs2::BufferedValue<planners::BaseTrajectoryPlanner::BaseReferenceCommand> currentCommand_;

      using EndEffectorTrajectories = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::EndEffectorTrajectories;
      ocs2::BufferedValue<EndEffectorTrajectories> referenceTrajectories_;

      using FootTangentialConstraintTrajectories = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::FootTangentialConstraintTrajectories;
      ocs2::BufferedValue<FootTangentialConstraintTrajectories> footConstraintTrajectories_;
      
      std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel_;
      ocs2::BufferedPointer<terrain_model::TerrainModel> bufferedTerrainModel_;

      locomotion::GaitPlanner gaitPlanner_;
      locomotion::SwingTrajectoryPlanner swingPlanner_;
      planners::BaseTrajectoryPlanner basePlanner_;
      planners::JointTrajectoryPlanner jointPlanner_;
      planners::ContactForceWrenchTrajectoryPlanner forcePlanner_;
  };

  /**
   * Creates LeggedReferenceManager settings 
   * @param [in] filename: file path with swing planner settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return LeggedReferenceManager::Settings struct
   */
  LeggedReferenceManager::Settings loadLeggedReferenceManagerSettings(
    const std::string& filename,
    const std::string& fieldName = "reference_manager_settings",
    bool verbose = "true");
} // namespace legged_locomotion_mpc

#endif
