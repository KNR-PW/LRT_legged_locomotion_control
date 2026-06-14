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

/* In order to fix compilation error with 
 * partial specialization of X after instantiation of Y"
 */
#include <pinocchio/fwd.hpp>
#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/spatial/log.hpp>

#include <legged_locomotion_mpc/cost/TrajectoryTrackingCost.hpp>

#include <stdexcept>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  namespace cost
  {
    using namespace ocs2;
    using namespace floating_base_model;

    TrajectoryTrackingCost::TrajectoryTrackingCost(FloatingBaseModelInfo info,
      const LeggedReferenceManager& referenceManager,
      BaseWeights baseWeights, JointWeights jointWeights,
      EndEffectorWeights endEffectorWeights, const std::string& modelFolder,
      bool recompileLibraries, bool verbose):
      info_(std::move(info)), referenceManager_(referenceManager),
      baseWeights_(std::move(baseWeights)), jointWeights_(std::move(jointWeights)),
      endEffectorWeights_(std::move(endEffectorWeights)) 
    {

      if((baseWeights_.position.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base positon weights vector element smaller than 0!");
      }
     
      if((baseWeights_.rotation.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base rotation weights vector element smaller than 0!");
      }

      if((baseWeights_.linearVelocity.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base linear velocity weights vector element smaller than 0!");
      }

      if((baseWeights_.angularVelocity.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base angular velocity weights vector element smaller than 0!");
      }

      if((jointWeights_.positions.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Joint positon weights vector element smaller than 0!");
      }

      if(jointWeights_.positions.size() != info.actuatedDofNum)
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Joint positon weights vector wrong size!");
      }

      if((jointWeights_.velocities.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Joint velocity weights vector element smaller than 0!");
      }

      if(jointWeights_.velocities.size() != info.actuatedDofNum)
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Joint velocity weights vector wrong size!");
      }
      
      const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;
      
      if(endEffectorWeights_.positions.size() != endEffectorNum)
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Wrong size for end effectors position weights!");
      }

      if(endEffectorWeights_.velocities.size() != endEffectorNum)
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Wrong size for end effectors velocity weights!");
      }

      if(endEffectorWeights_.forces.size() != endEffectorNum)
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Wrong size for end effectors force weights!");
      }

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        const auto& position = endEffectorWeights_.positions[i];
        const auto& velocity = endEffectorWeights_.velocities[i];
        const auto& force = endEffectorWeights_.forces[i];
  
        if((position.array() < 0.0).any())
        {
          throw std::invalid_argument("[TrajectoryTrackingCost]: End effector position weights vector element smaller than 0!");
        }

        if((velocity.array() < 0.0).any())
        {
          throw std::invalid_argument("[TrajectoryTrackingCost]: End effector velocity weights vector element smaller than 0!");
        }

        if((force.array() < 0.0).any())
        {
          throw std::invalid_argument("[TrajectoryTrackingCost]: End effector force weights vector element smaller than 0!");
        }
      }

      auto systemFlowMapFunc = [&](const ad_vector_t& x, const ad_vector_t& p, 
        ad_vector_t& y) 
      {
        const Eigen::Matrix<ad_scalar_t, 3, 1> currentEulerAnglesAD = x;
        const Eigen::Matrix<ad_scalar_t, 3, 1> targetEulerAnglesAD = p;
        y = getLog3CppAd(currentEulerAnglesAD, targetEulerAnglesAD);
      };
    
      log3AdInterfacePtr_.reset(
          new CppAdInterface(systemFlowMapFunc, 3, 3, "log3", modelFolder));
    
      if(recompileLibraries) 
      {
        log3AdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      } 
      else 
      {
        log3AdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      }
    }

    TrajectoryTrackingCost* TrajectoryTrackingCost::clone() const
    {
      return new TrajectoryTrackingCost(*this);
    }

    scalar_t TrajectoryTrackingCost::getValue(scalar_t time, const vector_t& state, 
      const vector_t& input, const TargetTrajectories& targetTrajectories, 
      const PreComputation& preComp) const
    {
      scalar_t cost = 0.0;

      const auto targetState = targetTrajectories.getDesiredState(time);
      const auto targetInput = targetTrajectories.getDesiredInput(time);

      const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

      // Base current data
      const auto baseCurrentPosition = floating_base_model::
        access_helper_functions::getBasePosition(state, info_);

      const vector3_t baseCurrentEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(state, info_);

      const auto baseCurrentLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      const auto baseCurrentAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      // Base target data
      const auto baseTargetPosition = floating_base_model::
        access_helper_functions::getBasePosition(targetState, info_);

      const vector3_t baseTargetEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(targetState, info_);

      const auto baseTargetLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);

      const auto baseTargetAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);
      
      // Base error
      const vector3_t basePositionError = baseTargetPosition - baseCurrentPosition;

      const auto baseRotationError = log3AdInterfacePtr_->getFunctionValue(
        baseCurrentEulerAngles, baseTargetEulerAngles);

      const vector3_t baseLinearVelocityError = baseTargetLinearVelocity 
        - baseCurrentLinearVelocity;

      const vector3_t baseAngularVelocityError = baseTargetAngularVelocity 
        - baseCurrentAngularVelocity;
      
      // Add base weighted squared cost
      cost += basePositionError.dot(baseWeights_.position.asDiagonal() * basePositionError);
      cost += baseRotationError.dot(baseWeights_.rotation.asDiagonal() * baseRotationError);
      cost += baseLinearVelocityError.dot(baseWeights_.linearVelocity.asDiagonal() 
        * baseLinearVelocityError);
      cost += baseAngularVelocityError.dot(baseWeights_.angularVelocity.asDiagonal() 
        * baseAngularVelocityError);

      const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;

      const auto& endEffectorTargets = referenceManager_.getEndEffectorTrajectoryPoint(time);

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        // End effectors current data
        const vector3_t& endEffectorCurrentPositon = leggedPrecomputation.getEndEffectorPosition(i);
        const vector3_t& endEffectorCurrentVelocity = leggedPrecomputation.getEndEffectorLinearVelocity(i);
        const auto endEffectorCurrentForce = floating_base_model::
          access_helper_functions::getContactForces(input, i, info_);

        // End effectors target data
        const vector3_t& endEffectorTargetPositon = endEffectorTargets.positions[i];
        const vector3_t& endEffectorTargetVelocity = endEffectorTargets.velocities[i];

        // In order to get true contact forces, not interpolated ones (when contact is changed for example)
        const auto& endEffectorTargetWrench = 
          leggedPrecomputation.getEndEffectorCompensationContactWrench(i);

        const vector3_t endEffectorTargetForce = endEffectorTargetWrench.block<3, 1>(0, 0);
        
        // End effectors error
        const vector3_t endEffectorPositionError = endEffectorTargetPositon - endEffectorCurrentPositon;
        const vector3_t endEffectorVelocityError = endEffectorTargetVelocity - endEffectorCurrentVelocity;
        const vector3_t endEffectorForceError = endEffectorTargetForce - endEffectorCurrentForce;
        
        // Add end effectors weighted squared cost
        cost += endEffectorPositionError.dot(endEffectorWeights_.positions[i].asDiagonal() 
          * endEffectorPositionError);
        cost += endEffectorVelocityError.dot(endEffectorWeights_.velocities[i].asDiagonal() 
          * endEffectorVelocityError);
        cost += endEffectorForceError.dot(endEffectorWeights_.forces[i].asDiagonal() 
          * endEffectorForceError);

        if(i >= info_.numThreeDofContacts)
        {
          // Add also contact torque
          const auto endEffectorCurrentTorque = floating_base_model::
            access_helper_functions::getContactTorques(input, i, info_);

          const vector3_t endEffectorTargetTorque = endEffectorTargetWrench.block<3, 1>(3, 0);
          
          const vector3_t endEffectorTorqueError = endEffectorTargetTorque - endEffectorCurrentTorque;

          const vector3_t weightedEndEffectorTorqueError = 
            endEffectorWeights_.torques[i - info_.numThreeDofContacts].asDiagonal() * endEffectorTorqueError;

          cost += endEffectorTorqueError .dot(weightedEndEffectorTorqueError);
        }
      }

      // Joint current data
      const auto jointCurrentPositions = floating_base_model::
        access_helper_functions::getJointPositions(state, info_);
      
      const auto jointCurrentVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(input, info_);

      // Joint target data
      const auto jointTargetPositions = floating_base_model::
        access_helper_functions::getJointPositions(targetState, info_);
      
      const auto jointTargetVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(targetInput, info_);

      // Joint error
      const vector_t jointPositionError = jointTargetPositions - jointCurrentPositions;
      const vector_t jointVelocityError = jointTargetVelocities - jointCurrentVelocities;

      // Add joint weighted squared cost
      cost += jointPositionError.dot(jointWeights_.positions.asDiagonal() 
        * jointPositionError);
      cost += jointVelocityError.dot(jointWeights_.positions.asDiagonal() 
        * jointVelocityError);
      
      // 1 / 2 of sum
      return 0.5 * cost;
    }

    ScalarFunctionQuadraticApproximation TrajectoryTrackingCost::getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComp) const
    {
      ScalarFunctionQuadraticApproximation cost;

      cost.f = 0.0;
      cost.dfdx = vector_t::Zero(info_.stateDim);
      cost.dfdu = vector_t::Zero(info_.inputDim);
      cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
      cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);
      cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);
      
      const auto targetState = targetTrajectories.getDesiredState(time);
      const auto targetInput = targetTrajectories.getDesiredInput(time);

      const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

      // Base current data
      const auto baseCurrentPosition = floating_base_model::
        access_helper_functions::getBasePosition(state, info_);

      const vector3_t baseCurrentEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(state, info_);

      const auto baseCurrentLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      const auto baseCurrentAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      // Base target data
      const auto baseTargetPosition = floating_base_model::
        access_helper_functions::getBasePosition(targetState, info_);

      const vector3_t baseTargetEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(targetState, info_);

      const auto baseTargetLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);

      const auto baseTargetAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);
      
      // Base error
      const vector3_t basePositionError = baseTargetPosition - baseCurrentPosition;

      const auto baseRotationError = log3AdInterfacePtr_->getFunctionValue(
        baseCurrentEulerAngles, baseTargetEulerAngles);

      const vector3_t baseLinearVelocityError = baseTargetLinearVelocity 
        - baseCurrentLinearVelocity;

      const vector3_t baseAngularVelocityError = baseTargetAngularVelocity 
        - baseCurrentAngularVelocity;
      
      // Add base weighted squared cost and its derivatives
      const vector3_t weightedBasePositonError = baseWeights_.position.asDiagonal() * basePositionError;
      cost.f += basePositionError.dot(weightedBasePositonError);
      cost.dfdx.block<3, 1>(6, 0).noalias() += -weightedBasePositonError;
      cost.dfdxx.block<3, 3>(6, 6).diagonal().noalias() += baseWeights_.position;
      
      const vector3_t weightedBaseRotationError = baseWeights_.rotation.asDiagonal() 
        * baseRotationError;
      cost.f += baseRotationError.dot(weightedBaseRotationError);
      const matrix_t log3Derivative = log3AdInterfacePtr_->getJacobian(
        baseCurrentEulerAngles, baseTargetEulerAngles);
      cost.dfdx.block<3, 1>(9, 0).noalias() += log3Derivative.transpose() * weightedBaseRotationError;
      cost.dfdxx.block<3, 3>(9, 9).noalias() += log3Derivative.transpose() 
        * baseWeights_.rotation.asDiagonal() * log3Derivative;
      
      const vector3_t weightedBaseLinearVelocityError = 
        baseWeights_.linearVelocity.asDiagonal() * baseLinearVelocityError;
      cost.f += baseLinearVelocityError.dot(weightedBaseLinearVelocityError);
      cost.dfdx.block<3, 1>(0, 0).noalias() += -weightedBaseLinearVelocityError;
      cost.dfdxx.block<3, 3>(0, 0).diagonal().noalias() += baseWeights_.linearVelocity;
      
      const vector3_t weightedBaseAngularVelocityError = 
        baseWeights_.angularVelocity.asDiagonal() * baseAngularVelocityError;
      cost.f += baseAngularVelocityError.dot(weightedBaseAngularVelocityError);
      cost.dfdx.block<3, 1>(3, 0).noalias() += -weightedBaseAngularVelocityError;
      cost.dfdxx.block<3, 3>(3, 3).diagonal().noalias() += baseWeights_.angularVelocity;

      const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;

      const auto& endEffectorTargets = referenceManager_.getEndEffectorTrajectoryPoint(time);

      const size_t forceIndexOffset = 3 * info_.numThreeDofContacts 
          + 6 * info_.numSixDofContacts;

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        // End effectors current data
        const vector3_t& endEffectorCurrentPositon = leggedPrecomputation.getEndEffectorPosition(i);
        const auto& endEffectorCurrentPositonDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);

        const vector3_t& endEffectorCurrentVelocity = leggedPrecomputation.getEndEffectorLinearVelocity(i);
        const auto& endEffectorCurrentVelocityDerivative = leggedPrecomputation.getEndEffectorLinearVelocityDerivatives(i);

        const auto endEffectorCurrentForce = floating_base_model::
          access_helper_functions::getContactForces(input, i, info_);

        // End effectors target data
        const vector3_t& endEffectorTargetPositon = endEffectorTargets.positions[i];
        const vector3_t& endEffectorTargetVelocity = endEffectorTargets.velocities[i];

        // In order to get true contact forces, not interpolated ones (when contact is changed for example)
        const auto& endEffectorTargetWrench = 
          leggedPrecomputation.getEndEffectorCompensationContactWrench(i);

        const vector3_t endEffectorTargetForce = endEffectorTargetWrench.block<3, 1>(0, 0);
        
        // End effectors error
        const vector3_t endEffectorPositionError = endEffectorTargetPositon - endEffectorCurrentPositon;
        const vector3_t endEffectorVelocityError = endEffectorTargetVelocity - endEffectorCurrentVelocity;
        const vector3_t endEffectorForceError = endEffectorTargetForce - endEffectorCurrentForce;

        const vector3_t weightedEndEffectorPositionError = 
          endEffectorWeights_.positions[i].asDiagonal() * endEffectorPositionError;

        const vector3_t weightedEndEffectorVelocityError = 
          endEffectorWeights_.velocities[i].asDiagonal() * endEffectorVelocityError;

        const vector3_t weightedEndEffectorForceError = 
          endEffectorWeights_.forces[i].asDiagonal() * endEffectorForceError;

        const matrix_t positionDerivativeBlock = 
          endEffectorCurrentPositonDerivative.dfdx.rightCols(info_.stateDim - 6);

        const matrix_t velocityDerivativeBlock = 
          endEffectorCurrentVelocityDerivative.dfdu.rightCols(info_.actuatedDofNum);

        // Add end effectors weighted squared cost and its derivatives
        cost.f += endEffectorPositionError.dot(weightedEndEffectorPositionError);
        cost.dfdx.bottomRows(info_.stateDim - 6).noalias() += 
          -positionDerivativeBlock.transpose() * weightedEndEffectorPositionError;
        cost.dfdxx.block(6, 6, info_.stateDim - 6, info_.stateDim - 6).noalias() += 
          positionDerivativeBlock.transpose() * endEffectorWeights_.positions[i].asDiagonal() 
          * positionDerivativeBlock;

        cost.f += endEffectorVelocityError.dot(weightedEndEffectorVelocityError);
        cost.dfdx.noalias() += -endEffectorCurrentVelocityDerivative.dfdx.transpose() 
          * weightedEndEffectorVelocityError;
        cost.dfdu.bottomRows(info_.actuatedDofNum).noalias() += 
          -velocityDerivativeBlock.transpose() * weightedEndEffectorVelocityError;
        cost.dfdxx.noalias() += endEffectorCurrentVelocityDerivative.dfdx.transpose() * 
          endEffectorWeights_.velocities[i].asDiagonal() * endEffectorCurrentVelocityDerivative.dfdx;
        cost.dfduu.block(forceIndexOffset, forceIndexOffset, 
          info_.actuatedDofNum, info_.actuatedDofNum).noalias() += 
            velocityDerivativeBlock.transpose() 
            * endEffectorWeights_.velocities[i].asDiagonal() * velocityDerivativeBlock;
        cost.dfdux.bottomRows(info_.actuatedDofNum).noalias() += 
          velocityDerivativeBlock.transpose() 
            * endEffectorWeights_.velocities[i].asDiagonal() * endEffectorCurrentVelocityDerivative.dfdx;

        cost.f += endEffectorForceError.dot(weightedEndEffectorForceError);
        if(i < info_.numThreeDofContacts)
        {
          cost.dfdu.block<3, 1>(3 * i, 0).noalias() += -weightedEndEffectorForceError;
          cost.dfduu.block<3, 3>(3 * i, 3 * i).diagonal().noalias() += 
            endEffectorWeights_.forces[i];
        }
        else
        {
          // Add also contact torque
          const auto endEffectorCurrentTorque = floating_base_model::
            access_helper_functions::getContactTorques(input, i, info_);

          const vector3_t endEffectorTargetTorque = endEffectorTargetWrench.block<3, 1>(3, 0);
          
          const vector3_t endEffectorTorqueError = endEffectorTargetTorque - endEffectorCurrentTorque;

          const vector3_t weightedEndEffectorTorqueError = 
            endEffectorWeights_.torques[i - info_.numThreeDofContacts].asDiagonal() * endEffectorTorqueError;

          cost.f += endEffectorTorqueError .dot(weightedEndEffectorTorqueError);

          const size_t forceStartIndex = 6 * i - 3 * info_.numThreeDofContacts;
          const size_t torqueStartIndex = forceStartIndex + 3;

          cost.dfdu.block<3, 1>(forceStartIndex, 0).noalias() += -weightedEndEffectorForceError;
          cost.dfdu.block<3, 1>(torqueStartIndex, 0).noalias() += -weightedEndEffectorTorqueError;
          
          cost.dfduu.block<3, 3>(forceStartIndex, forceStartIndex).diagonal().noalias() += 
            endEffectorWeights_.forces[i];
          cost.dfduu.block<3, 3>(torqueStartIndex, torqueStartIndex).diagonal().noalias() += 
            endEffectorWeights_.torques[i - info_.numThreeDofContacts];
        }
      }

      // Joint current data
      const auto jointCurrentPositions = floating_base_model::
        access_helper_functions::getJointPositions(state, info_);
      
      const auto jointCurrentVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(input, info_);

      // Joint target data
      const auto jointTargetPositions = floating_base_model::
        access_helper_functions::getJointPositions(targetState, info_);
      
      const auto jointTargetVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(targetInput, info_);

      // Joint error
      const vector_t jointPositionError = jointTargetPositions - jointCurrentPositions;
      const vector_t jointVelocityError = jointTargetVelocities - jointCurrentVelocities;

      const vector_t weightedJointPositionError = jointWeights_.positions.asDiagonal() 
        * jointPositionError;

      const vector_t weightedJointVelocityError = jointWeights_.velocities.asDiagonal() 
        * jointVelocityError;

      // Add joint weighted squared cost and its derivatives
      cost.f += jointPositionError.dot(weightedJointPositionError);
      cost.dfdx.block(12, 0, info_.actuatedDofNum, 1).noalias() += -weightedJointPositionError;
      cost.dfdxx.block(12, 12, info_.actuatedDofNum, info_.actuatedDofNum).diagonal().noalias() += 
        jointWeights_.positions;

      cost.f += jointVelocityError.dot(weightedJointVelocityError);
      cost.dfdu.block(forceIndexOffset, 0, info_.actuatedDofNum, 1).noalias() += 
        -weightedJointVelocityError;

      cost.dfduu.block(forceIndexOffset, forceIndexOffset, info_.actuatedDofNum, 
        info_.actuatedDofNum).diagonal().noalias() += jointWeights_.velocities;
      
      // 1 / 2 of sum
      cost.f *= 0.5;

      return cost;
    }

    TrajectoryTrackingCost::TrajectoryTrackingCost(const TrajectoryTrackingCost& rhs):
      info_(rhs.info_), referenceManager_(rhs.referenceManager_), 
      baseWeights_(rhs.baseWeights_), jointWeights_(rhs.jointWeights_),
      endEffectorWeights_(rhs.endEffectorWeights_), 
      log3AdInterfacePtr_(new CppAdInterface(*rhs.log3AdInterfacePtr_)) {}

    ad_vector_t TrajectoryTrackingCost::getLog3CppAd(
      const Eigen::Matrix<ad_scalar_t, 3, 1>& currentEulerAngles, 
      const Eigen::Matrix<ad_scalar_t, 3, 1>& targetEulerAngles)
    {
      const auto targetRotation = getRotationMatrixFromZyxEulerAngles(targetEulerAngles);
      const auto currentRotation = getRotationMatrixFromZyxEulerAngles(currentEulerAngles);
      return pinocchio::log3(targetRotation.transpose() * currentRotation);
    }

    TrajectoryTrackingCost::BaseWeights loadBaseWeights(const std::string& filename,
      const std::string& fieldName, bool verbose)
    {

      boost::property_tree::ptree pt;
      read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Base Weights:";
        std::cerr << "\n #### =============================================================================\n";
      }
      
      TrajectoryTrackingCost::BaseWeights baseWeights;

      loadData::loadEigenMatrix(filename, fieldName + ".position", baseWeights.position);
      if((baseWeights.position.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base positon weights vector element smaller than 0!");
      }
     
      loadData::loadEigenMatrix(filename, fieldName + ".rotation", baseWeights.rotation);
      if((baseWeights.rotation.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base rotation weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".linearVelocity", baseWeights.linearVelocity);
      if((baseWeights.linearVelocity.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base linear velocity weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".angularVelocity", baseWeights.angularVelocity);
      if((baseWeights.angularVelocity.array() < 0.0).any())
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Base angular velocity weights vector element smaller than 0!");
      }
    
      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return baseWeights;
    }
 
    TrajectoryTrackingCost::JointWeights loadJointWeights(const std::string& filename,
      const FloatingBaseModelInfo& info, const pinocchio::Model& robotModel,
      const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Joint Weights:";
        std::cerr << "\n #### =============================================================================\n";
      }
      
      TrajectoryTrackingCost::JointWeights jointWeights;

      std::vector<std::string> jointNames = robotModel.names;

      // Remove "universe" and "root_joint" joints from joint names
      jointNames.erase(std::remove(jointNames.begin(), jointNames.end(), "universe"), jointNames.end());
      jointNames.erase(std::remove(jointNames.begin(), jointNames.end(), "root_joint"), jointNames.end()); 

      if(jointNames.size() != info.actuatedDofNum)
      {
        throw std::invalid_argument("[TrajectoryTrackingCost]: Wrong size of joint names vector!");
      }

      jointWeights.positions = Eigen::VectorXd::Zero(info.actuatedDofNum);
      jointWeights.velocities = Eigen::VectorXd::Zero(info.actuatedDofNum);

      for(size_t i = 0; i < info.actuatedDofNum; ++i)
      {
        const std::string& jointName = jointNames[i];
        
        ocs2::loadData::loadPtreeValue(pt, jointWeights.positions[i], fieldName + "." + jointName + ".position", verbose);
        if(jointWeights.positions[i] < 0.0)
        {
          std::string message = "[TrajectoryTrackingCost]: Joint positon weights element for joint " + jointName + " smaller than 0!";
          throw std::invalid_argument(message);
        }

        ocs2::loadData::loadPtreeValue(pt, jointWeights.velocities[i], fieldName + "." + jointName + ".velocity", verbose);
        if(jointWeights.velocities[i] < 0.0)
        {
          std::string message = "[TrajectoryTrackingCost]: Joint velocity weights element for joint " + jointName + " smaller than 0!";
          throw std::invalid_argument(message);
        }
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return jointWeights;
    }

    TrajectoryTrackingCost::EndEffectorWeights loadEndEffectorWeights(
      const std::string& filename, const ModelSettings& modelSettings, 
      const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      read_info(filename, pt);

      TrajectoryTrackingCost::EndEffectorWeights endEffectorWeights;

      const size_t endEffectorNum = modelSettings.endEffectorThreeDofNames.size() + modelSettings.endEffectorSixDofNames.size();

      endEffectorWeights.positions.resize(endEffectorNum);
      endEffectorWeights.velocities.resize(endEffectorNum);
      endEffectorWeights.forces.resize(endEffectorNum);
      endEffectorWeights.torques.resize(modelSettings.endEffectorSixDofNames.size());

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC End Effector Weights:";
        std::cerr << "\n #### =============================================================================\n";
      }

      std::vector<std::reference_wrapper<const std::string>> endEffectorNames;

      for(const auto& threeDofContact: modelSettings.endEffectorThreeDofNames)
      {
        endEffectorNames.push_back(threeDofContact);
      }

      for(const auto& sixDofContact: modelSettings.endEffectorSixDofNames)
      {
        endEffectorNames.push_back(sixDofContact);
      }

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        const std::string& name = endEffectorNames[i].get();
        auto& position = endEffectorWeights.positions[i];
        auto& velocity = endEffectorWeights.velocities[i];
        auto& force = endEffectorWeights.forces[i];

        loadData::loadEigenMatrix(filename, fieldName + "." + name + ".position", position);
        if((position.array() < 0.0).any())
        {
          std::string message = "[TrajectoryTrackingCost]: " + name + " position weights vector element smaller than 0!";
          throw std::invalid_argument(message);
        }

        loadData::loadEigenMatrix(filename, fieldName + "." + name + ".velocity", velocity);
        if((velocity.array() < 0.0).any())
        {
          std::string message = "[TrajectoryTrackingCost]: " + name + " velocity weights vector element smaller than 0!";
          throw std::invalid_argument(message);
        }

        loadData::loadEigenMatrix(filename, fieldName + "." + name + ".force", force);
        if((force.array() < 0.0).any())
        {
          std::string message = "[TrajectoryTrackingCost]: " + name + " force weights vector element smaller than 0!";
          throw std::invalid_argument(message);
        }

        // Only for 6 DoF end effectors
        if(endEffectorNum > modelSettings.endEffectorThreeDofNames.size())
        {
          auto& torque = endEffectorWeights.torques[i - modelSettings.endEffectorThreeDofNames.size()];
          loadData::loadEigenMatrix(filename, fieldName + "." + name + ".torque", torque);
          if((force.array() < 0.0).any())
          {
            std::string message = "[TrajectoryTrackingCost]: " + name + " torque weights vector element smaller than 0!";
            throw std::invalid_argument(message);
          }
        }
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }
      
      return endEffectorWeights;
    }
  } // namespace cost
} // namespace legged_locomotion_mpc