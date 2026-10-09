//
// Created by qiayuan on 2022/7/1.
//

// Copyright (c) 2026, Bartłomiej Krajewski
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
 * Modified by: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __LEGGED_WHOLE_BODY_CONTROL_WBC_BASE__
#define __LEGGED_WHOLE_BODY_CONTROL_WBC_BASE__

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_whole_body_control/Task.h>

namespace legged_whole_body_control 
{
  // Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
  class WbcBase 
  {
    public:

      struct BaseTrackingTaskSettings
      {
        vector3_t linearFeedForwardGain;
        vector3_t linearProportionalGain;
        vector3_t linearDerivativeGain;
      
        vector3_t angularFeedForwardGain;
        vector3_t angularProportionalGain;
        vector3_t angularDerivativeGain;
      };
    
      struct EndEffectorsTrackingTaskSettings
      {
        // For 3 DoF end-effectors
        std::vector<vector3_t> linearFeedForwardGain;
        std::vector<vector3_t> linearProportionalGain;
        std::vector<vector3_t> linearDerivativeGain;
      
        // For 6 DoF end-effectors
        std::vector<vector3_t> angularFeedForwardGain;
        std::vector<vector3_t> angularProportionalGain;
        std::vector<vector3_t> angularDerivativeGain;
      };
    
      struct FrictionConeTaskSettings
      {
        ocs2::scalar_t frictionCoefficient = 0.5;
      };
    
      struct Settings
      {
        std::string worldLinkName;
        std::string baseLinkName;
      
        std::vector<std::string> endEffectorThreeDofNames;
        std::vector<std::string> endEffectorSixDofNames;

        bool useDynamicsTask = true;
        bool useBaseTrackingTask = true;
        bool useEndEffectorsTrackingTask = true;
        bool useContactForceTrackingTask = true;
        bool useTorqueLimitsTask = true;
        bool useKinematicContactTask = true;
        bool useFrictionConeTask = true;
      
        BaseTrackingTaskSettings baseSettings;
        EndEffectorsTrackingTaskSettings endEffectorSettings;
        FrictionConeTaskSettings frictionConeSettings;

        ocs2::scalar_t desiredFrequency;
      };

      WbcBase(const ocs2::PinocchioInterface& pinocchioInterface, 
        floating_base_model::FloatingBaseModelInfo info, 
        Settings settings);

      void updateDesired(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      void updateMeasured(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      void updateContactFlags(ocs2::scalar_t time, const contact_flags_t& contactFlags);

      void updateTerrainNormals(ocs2::scalar_t time, const std::vector<vector3_t>& normals);

      void updateExternalWrench(ocs2::scalar_t time, const vector6_t& externalBaseWrench);

      Eigen::Ref<const vector6_t> getBaseAcceleration() const;

      Eigen::Ref<const ocs2::vector_t> getJointAcceleration() const;

      Eigen::Ref<const ocs2::vector_t> getJointTorque() const;

      Eigen::Ref<const vector3_t> getEndEffectorForce(size_t endEffectorIndex) const;

      Eigen::Ref<const vector6_t> getEndEffectorWrench(size_t getEndEffectorIndex) const;

      virtual void calculate(ocs2::scalar_t time);

   protected:

    void calculateMeasured();

    void calculateDesired();

    Task formulateDynamicsTask() const;
    Task formulateTorqueLimitsTask() const;
    Task formulateKinematicContactTask() const;
    Task formulateFrictionConeTask() const;
    Task formulateBaseTrackingTask() const;
    Task formulateEndEffectorsTrackingTask() const;
    Task formulateContactForceTrackingTask() const;

    size_t numberOfDecisionVariables_;
    
    Settings settings_;
    floating_base_model::FloatingBaseModelInfo info_;
    
    floating_base_model::FloatingBaseModelPinocchioMapping mappingMeasured_;
    floating_base_model::FloatingBaseModelPinocchioMapping mappingDesired_;
    ocs2::PinocchioInterface pinocchioInterfaceMeasured_;
    ocs2::PinocchioInterface pinocchioInterfaceDesired_;

    ocs2::scalar_t timeMeasured_;
    ocs2::vector_t stateMeasured_;
    ocs2::vector_t inputMeasured_;

    bool desiredCached_ = false;
    ocs2::scalar_t timeDesired_;
    ocs2::scalar_t previousTimeDesired_;
    ocs2::vector_t stateDesired_;
    ocs2::vector_t inputDesired_;
    ocs2::vector_t previousStateDesired_;
    ocs2::vector_t previousInputDesired_;

    ocs2::scalar_t timeContact_;
    contact_flags_t contactFlags_;

    ocs2::scalar_t timeNormals_;
    std::vector<vector3_t> terrainNormals_;

    ocs2::scalar_t timeExternalWrench_;
    vector6_t externalWrench_;

    vector6_t feedFrowardBaseAcceleration_;

    ocs2::matrix_t stackedJacobians_;
    ocs2::matrix_t stackedJacobianDerivatives_;

    ocs2::vector_t currentResult_;

    ocs2::matrix_t jointActuationMatrix_;
  };

  /**
   * Creates WbcBase settings
   * @param [in] filename: file path with wbc base settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return WbcBaseSettings struct
   */
  WbcBase::Settings loadWbcBaseSettings(const std::string& filename,
    const std::string& fieldName = "wbc_base_settings",
    bool verbose = "true");
}; // namespace legged_whole_body_control
#endif