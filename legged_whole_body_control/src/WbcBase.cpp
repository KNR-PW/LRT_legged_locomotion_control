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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <legged_whole_body_control/WbcBase.hpp>

#include <utility>
#include <unordered_set>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>

namespace legged_whole_body_control  
{
  using namespace ocs2;
  using namespace floating_base_model;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, 
    FloatingBaseModelInfo info, WbcBase::Settings settings):
      settings_(std::move(settings)), info_(info), 
      mappingMeasured_(info), 
      mappingDesired_(info), 
      pinocchioInterfaceMeasured_(pinocchioInterface), 
      pinocchioInterfaceDesired_(pinocchioInterface)
  {
    numberOfDecisionVariables_ = info_.generalizedCoordinatesNum;
    numberOfDecisionVariables_ += 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
    numberOfDecisionVariables_ += info_.actuatedDofNum;

    timeMeasured_ = 0.0;
    stateMeasured_ = vector_t::Zero(info_.stateDim);
    inputMeasured_ = vector_t::Zero(info_.inputDim);

    timeDesired_ = 0.0;
    previousTimeDesired_ = 0.0;
    stateDesired_ = vector_t::Zero(info_.stateDim);
    inputDesired_ = vector_t::Zero(info_.inputDim);
    previousInputDesired_ = vector_t::Zero(info_.inputDim);

    stackedJacobians_ = matrix_t::Zero(3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts, 
      info_.generalizedCoordinatesNum);
    stackedJacobianDerivatives_ = matrix_t::Zero(3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts, 
      info_.generalizedCoordinatesNum);

    jointActuationMatrix_ = matrix_t(info_.actuatedDofNum, 
      info_.generalizedCoordinatesNum);
    jointActuationMatrix_.block(0, 0, info_.actuatedDofNum, 6).setZero();
    jointActuationMatrix_.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
    jointActuationMatrix_ = jointActuationMatrix_.transpose();

    mappingMeasured_.setPinocchioInterface(pinocchioInterfaceMeasured_);
    mappingDesired_.setPinocchioInterface(pinocchioInterfaceDesired_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::updateDesired(scalar_t time, const vector_t& state, 
    const vector_t& input)
  {
    assert(state.size() == info_.stateDim);
    assert(input.size() == info_.inputDim);

    if(desiredCached_)
    {
      previousTimeDesired_ = timeDesired_;
      timeDesired_ = time;

      previousStateDesired_ = std::move(stateDesired_);
      stateDesired_ = state;

      previousInputDesired_ = std::move(inputDesired_);
      inputDesired_ = input;
    }
    else
    {
      desiredCached_ = true;

      timeDesired_ = time;
      previousTimeDesired_ = time;

      stateDesired_ = state;
      previousStateDesired_ = state;

      inputDesired_ = input;
      previousInputDesired_ = input;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::updateMeasured(scalar_t time, const vector_t& state, 
    const vector_t& input)
  {
    assert(state.size() == info_.stateDim);
    assert(input.size() == info_.inputDim);

    timeMeasured_ = time;
    stateMeasured_ = state;
    inputMeasured_ = input;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::updateContactFlags(scalar_t time, 
    const contact_flags_t& contactFlags)
  {
    assert(contactFlags_.size() == (info_.numThreeDofContacts + info_.numSixDofContacts));

    timeContact_ = time;
    contactFlags_ = contactFlags;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::updateTerrainNormals(ocs2::scalar_t time, 
    const std::vector<vector3_t>& normals)
  {
    assert(terrainNormals_.size() == (info_.numThreeDofContacts + info_.numSixDofContacts));

    timeNormals_ = time;
    terrainNormals_ = normals;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::updateExternalWrench(ocs2::scalar_t time, 
    const vector6_t& externalBaseWrench)
  {
    timeExternalWrench_ = time;
    externalWrench_ = externalBaseWrench;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Ref<const vector6_t> WbcBase::getBaseAcceleration() const
  {
    return currentResult_.head<6>();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Ref<const vector_t> WbcBase::getJointAcceleration() const
  {
    return currentResult_.block(6, 0, info_.actuatedDofNum, 1);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Ref<const vector_t> WbcBase::getJointTorque() const
  {
    return currentResult_.tail(info_.actuatedDofNum);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Ref<const vector3_t> WbcBase::getEndEffectorForce(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < info_.numThreeDofContacts);
    size_t contactStartIndex = info_.generalizedCoordinatesNum + 3 * endEffectorIndex;
    return currentResult_.block<3, 1>(contactStartIndex, 0);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Ref<const vector6_t> WbcBase::getEndEffectorWrench(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex >= info_.numThreeDofContacts);
    assert(endEffectorIndex < info_.numThreeDofContacts + info_.numSixDofContacts);
    size_t contactStartIndex = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + 6 * (endEffectorIndex - info_.numThreeDofContacts);
    return currentResult_.block<6, 1>(contactStartIndex, 0);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::calculate(scalar_t time)
  {
    calculateMeasured();
    calculateDesired();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::calculateMeasured() 
  {
    const vector_t generalizedPositions = mappingMeasured_.getPinocchioJointPosition(stateMeasured_);
    const vector_t generalizedVelocities = mappingMeasured_.getPinocchioJointVelocity(stateMeasured_, 
      inputMeasured_);

    const auto& model = pinocchioInterfaceMeasured_.getModel();
    auto& data = pinocchioInterfaceMeasured_.getData();

    pinocchio::forwardKinematics(model, data, generalizedPositions, generalizedVelocities);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, generalizedPositions);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, generalizedPositions, generalizedVelocities);
    
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      stackedJacobians_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    for (size_t i = info_.numThreeDofContacts; 
      i < (info_.numThreeDofContacts + info_.numSixDofContacts); ++i) 
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      size_t startIndex = 3 * info_.numThreeDofContacts + 6 * (i - info_.numThreeDofContacts);
      stackedJacobians_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum) = jac;
    }

    pinocchio::computeJointJacobiansTimeVariation(model, data, generalizedPositions, 
      generalizedVelocities);

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      stackedJacobianDerivatives_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    for (size_t i = info_.numThreeDofContacts; 
      i < (info_.numThreeDofContacts + info_.numSixDofContacts); ++i) 
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      size_t startIndex = 3 * info_.numThreeDofContacts + 6 * (i - info_.numThreeDofContacts);
      stackedJacobianDerivatives_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum) = jac;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WbcBase::calculateDesired() 
  {
    const auto& model = pinocchioInterfaceDesired_.getModel();
    auto& data = pinocchioInterfaceDesired_.getData();

    const vector_t generalizedPosition = mappingDesired_.getPinocchioJointPosition(stateDesired_);
    const vector_t currentVelocities = mappingDesired_.getPinocchioJointVelocity(stateDesired_, inputDesired_);
    const vector_t previousVelocities = mappingDesired_.getPinocchioJointVelocity(
      previousStateDesired_, previousInputDesired_);

    const vector_t generalizedAcceleration = (currentVelocities - previousVelocities) / (timeDesired_ - previousTimeDesired_);

    feedFrowardBaseAcceleration_ = generalizedAcceleration.block<6, 1>(0, 0);

    pinocchio::forwardKinematics(model, data, generalizedPosition, currentVelocities, 
      generalizedAcceleration);
    pinocchio::computeJointJacobians(model, data, generalizedPosition);
    pinocchio::updateFramePlacements(model, data);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateDynamicsTask() const
  {
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numberOfDecisionVariables_) 
      << data.M, -stackedJacobians_.transpose(), -jointActuationMatrix_).finished();
    vector_t b = -data.nle;
    b.segment<6>(0) += externalWrench_; 

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateTorqueLimitsTask() const
  {
    matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

    matrix_t d(2 * info_.actuatedDofNum, numberOfDecisionVariables_);
    d.setZero();
    const size_t startIndex = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
    d.block(0, startIndex, info_.actuatedDofNum, info_.actuatedDofNum) = i;
    d.block(info_.actuatedDofNum, startIndex, info_.actuatedDofNum, 
      info_.actuatedDofNum) = -i;

    const auto& model = pinocchioInterfaceDesired_.getModel();
    const vector_t jointMaxTorque =  model.effortLimit.segment(6, info_.actuatedDofNum);

    vector_t f(2 * info_.actuatedDofNum);
    f << jointMaxTorque, jointMaxTorque;
    
    return Task(matrix_t(), vector_t(), std::move(d), std::move(f));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateKinematicContactTask() const
  {
    const vector_t generalizedVelocities = mappingMeasured_.getPinocchioJointVelocity(stateMeasured_, 
      inputMeasured_);

    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    matrix_t a(3 * numberOf3DofContacts + 6 * numberOf6DofContacts, 
      numberOfDecisionVariables_);
    vector_t b(a.rows());

    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) 
    {
      if(contactFlags_[i]) 
      {
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = stackedJacobians_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment<3>(3 * j) = -stackedJacobianDerivatives_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * generalizedVelocities;
        j++;
      }
    }
    
    j *= 3;
    size_t k = 0;
    for (size_t i = info_.numThreeDofContacts; 
      i < info_.numThreeDofContacts + info_.numSixDofContacts; i++) 
    {
      if(contactFlags_[i]) 
      {
        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;
        a.block(6 * k + j, 0, 6, info_.generalizedCoordinatesNum) = stackedJacobians_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum);
        b.segment<6>(6 * k + j) = -stackedJacobianDerivatives_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum) * generalizedVelocities;
        k++;
      }
    }

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateFrictionConeTask() const
  {
    // No contact == zero force/wrench
    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    matrix_t a(3 * (info_.numThreeDofContacts - numberOf3DofContacts) 
      + 6 * (info_.numSixDofContacts - numberOf6DofContacts), 
      numberOfDecisionVariables_);
    a.setZero();

    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if(!contactFlags_[i]) 
      {
        a.block(3 * j, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix3_t::Identity();
        j++;
      }
    }

    j *= 3;
    size_t k = 0;
    for (size_t i = info_.numThreeDofContacts; 
      i < info_.numThreeDofContacts + info_.numSixDofContacts; i++)
    {
      if(!contactFlags_[i]) 
      {
        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;
        a.block(6 * k + j, info_.generalizedCoordinatesNum + startIndex, 6, 6) = matrix6_t::Identity();
        k++;
      }
    }

    vector_t b(a.rows());
    b.setZero();

    // Calculate heading and lateral vectors:
    const vector3_t baseEulerAngles = access_helper_functions::getBaseOrientationZyx(
      stateMeasured_, info_);

    const matrix3_t baseRotationMatrix = getRotationMatrixFromZyxEulerAngles(baseEulerAngles);

    const vector3_t baseHeading =  baseRotationMatrix.col(0);

    matrix_t d(5 * (numberOf3DofContacts + numberOf6DofContacts), numberOfDecisionVariables_);
    d.setZero();

    j = 0;
    for(size_t i = 0; i < info_.numThreeDofContacts + info_.numSixDofContacts; ++i)
    {
      if(contactFlags_[i]) 
      {
        const vector3_t normal = [&](){
        if(terrainNormals_.size() == 0)
        {
          return terrainNormals_[i].normalized();
        }
        else
        {
          return vector3_t(0.0, 0.0, 1.0);
        }}();
        const vector3_t heading = (baseHeading - normal.dot(baseHeading) * normal).normalized();
        const vector3_t lateral = normal.cross(heading);
        const vector3_t frictionNormal = settings_.frictionConeSettings.frictionCoefficient * normal;
        
        Eigen::Matrix<scalar_t, 5, 3> frictionPyramic;
        frictionPyramic << -normal.transpose(), 
                           (heading - frictionNormal).transpose(),
                           -(heading + frictionNormal).transpose(),
                           (lateral - frictionNormal).transpose(),
                           -(lateral + frictionNormal).transpose();

        const size_t offset = [&]()
        {
          if(i < info_.numThreeDofContacts)
          {
            return info_.generalizedCoordinatesNum + 3 * i;
          }
          else
          {
            return info_.generalizedCoordinatesNum + 6 * i - 3 * info_.numThreeDofContacts;
          }
        }();

        d.block(5 * j++, offset, 5, 3) = frictionPyramic;
      }
    }

    vector_t f = Eigen::VectorXd::Zero(d.rows());

    return Task(std::move(a), std::move(b), std::move(d), std::move(f));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateBaseTrackingTask() const
  {
    matrix_t a(6, numberOfDecisionVariables_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix6_t::Identity();

    const auto& baseSettings = settings_.baseSettings;

    const auto basePositionMeasured = access_helper_functions::getBasePosition(
      stateMeasured_, info_);
    const auto basePositionDesired = access_helper_functions::getBasePosition(
      stateDesired_, info_);

    const vector3_t baseEulerMeasured = access_helper_functions::getBaseOrientationZyx(
      stateMeasured_, info_);
    const vector3_t baseEulerDesired = access_helper_functions::getBaseOrientationZyx(
      stateDesired_, info_);

    const auto targetRotation = getRotationMatrixFromZyxEulerAngles(baseEulerDesired);
    const auto currentRotation = getRotationMatrixFromZyxEulerAngles(baseEulerMeasured);

    const auto baseLinearVelocityMeasured = access_helper_functions::getBaseLinearVelocity(
      stateMeasured_, info_);
    const auto baseLinearVelocityDesired = access_helper_functions::getBaseLinearVelocity(
      stateDesired_, info_);

    const auto baseAngularVelocityMeasured = access_helper_functions::getBaseAngularVelocity(
      stateMeasured_, info_);
    const auto baseAngularVelocityDesired = access_helper_functions::getBaseAngularVelocity(
      stateDesired_, info_);

    const vector3_t basePositionError = currentRotation.transpose() * (
      basePositionDesired - basePositionMeasured);
    const vector3_t baseOrientationError = pinocchio::log3(currentRotation.transpose() * targetRotation);
    const vector3_t baseLinearVelocityError = baseLinearVelocityDesired - baseLinearVelocityMeasured;
    const vector3_t baseAngularVelocityError = baseAngularVelocityDesired - baseAngularVelocityMeasured;

    vector6_t b;
    b.segment<3>(0)  = baseSettings.linearFeedForwardGain.asDiagonal() * feedFrowardBaseAcceleration_.segment<3>(0);
    b.segment<3>(0) += baseSettings.linearProportionalGain.asDiagonal() * basePositionError;
    b.segment<3>(0) += baseSettings.linearDerivativeGain.asDiagonal() * baseLinearVelocityError;

    b.segment<3>(3)  = baseSettings.angularFeedForwardGain.asDiagonal() * feedFrowardBaseAcceleration_.segment<3>(3);
    b.segment<3>(3) += baseSettings.angularProportionalGain.asDiagonal() * baseOrientationError;
    b.segment<3>(3) += baseSettings.angularDerivativeGain.asDiagonal() * baseAngularVelocityError;
    
    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateEndEffectorsTrackingTask() const
  {
    const vector_t generalizedVelocities = mappingMeasured_.getPinocchioJointVelocity(
      stateMeasured_, inputMeasured_);

    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

    const auto& modelMeasured = pinocchioInterfaceMeasured_.getModel();
    auto& dataMeasured = pinocchioInterfaceMeasured_.getData();

    const auto& modelDesired = pinocchioInterfaceDesired_.getModel();
    auto& dataDesired  = pinocchioInterfaceDesired_.getData();

    const auto& endEffectorSettings = settings_.endEffectorSettings;

    matrix_t a(3 * (info_.numThreeDofContacts - numberOf3DofContacts) 
      + 6 * (info_.numSixDofContacts - numberOf6DofContacts), numberOfDecisionVariables_);

    vector_t b(a.rows());

    size_t j = 0;
    for(size_t i = 0; i < info_.numThreeDofContacts; ++i) 
    {
      if(!contactFlags_[i]) 
      {
        const size_t frameId = info_.endEffectorFrameIndices[i];

        const auto& ff = endEffectorSettings.linearFeedForwardGain[i];
        const auto& kp = endEffectorSettings.linearProportionalGain[i];
        const auto& kd = endEffectorSettings.linearDerivativeGain[i];

        const auto& positionDesired = dataDesired.oMf[frameId].translation();
        const vector3_t velocityDesired = pinocchio::getFrameVelocity(modelDesired, 
          dataDesired, frameId, rf).linear();

        const auto& positionMeasured = dataMeasured.oMf[frameId].translation();
        const vector3_t velocityMeasured = pinocchio::getFrameVelocity(modelMeasured, 
          dataMeasured, frameId, rf).linear();

        const vector3_t accelerationFeedForward = getFrameClassicalAcceleration(
          modelDesired, dataDesired, frameId, rf).linear();

        vector3_t acceleration = ff.asDiagonal() * accelerationFeedForward;
        acceleration += kp.asDiagonal() * (positionDesired - positionMeasured);
        acceleration += kd.asDiagonal() * (velocityDesired - velocityMeasured);
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = stackedJacobians_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = acceleration - stackedJacobianDerivatives_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * generalizedVelocities;
        j++;
      }
    }

    j *= 3;
    size_t k = 0;
    for(size_t i =  info_.numThreeDofContacts; 
      i < info_.numThreeDofContacts +  info_.numSixDofContacts; ++i) 
    {
      if(!contactFlags_[i]) 
      {
        const size_t frameId = info_.endEffectorFrameIndices[i];

        const auto& linearFf = endEffectorSettings.linearFeedForwardGain[i];
        const auto& linearKp = endEffectorSettings.linearProportionalGain[i];
        const auto& linearKd = endEffectorSettings.linearDerivativeGain[i];

        const auto& angularFf = endEffectorSettings.angularFeedForwardGain[i - info_.numThreeDofContacts];
        const auto& angularKp = endEffectorSettings.angularProportionalGain[i - info_.numThreeDofContacts];
        const auto& angularKd = endEffectorSettings.angularDerivativeGain[i - info_.numThreeDofContacts];

        const auto& positionDesired = dataDesired.oMf[frameId].translation();
        const auto& rotationDesired = dataDesired.oMf[frameId].rotation();
        const vector6_t velocityDesired = pinocchio::getFrameVelocity(modelDesired, 
          dataDesired, frameId, rf).toVector();

        const auto& positionMeasured = dataMeasured.oMf[frameId].translation();
        const auto& rotationMeasured = dataMeasured.oMf[frameId].rotation();
        const vector6_t velocityMeasured = pinocchio::getFrameVelocity(modelMeasured, 
          dataMeasured, frameId, rf).toVector();
        
        const vector3_t orientationError = rotationMeasured * pinocchio::log3(rotationMeasured.transpose() * rotationDesired);
        const vector6_t velocityError = velocityDesired - velocityMeasured;

        const vector6_t accelerationFeedForward = getFrameClassicalAcceleration(
          modelDesired, dataDesired, frameId, rf).toVector();

        vector6_t acceleration; 

        acceleration.segment<3>(0) =  linearFf.asDiagonal() * accelerationFeedForward.segment<3>(0);
        acceleration.segment<3>(0) += linearKp.asDiagonal() * (positionDesired - positionMeasured);
        acceleration.segment<3>(0) += linearKd.asDiagonal() * velocityError.segment<3>(0);

        acceleration.segment<3>(3) =  angularFf.asDiagonal() * accelerationFeedForward.segment<3>(3);
        acceleration.segment<3>(3) += angularKp.asDiagonal() * orientationError;
        acceleration.segment<3>(3) += angularKd.asDiagonal() * velocityError.segment<3>(3);

        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;

        a.block(6 * k + j, 0, 6, info_.generalizedCoordinatesNum) = stackedJacobians_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum);
        b.segment(6 * k + j, 6) = acceleration - stackedJacobianDerivatives_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum) * generalizedVelocities;
        k++;
      }
    }

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WbcBase::formulateContactForceTrackingTask() const 
  {
    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    matrix_t a(3 * numberOf3DofContacts + 6 * numberOf6DofContacts, 
      numberOfDecisionVariables_);

    vector_t b(a.rows());

    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
    {
      if(contactFlags_[i])
      {
        a.block(3 * j, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix3_t::Identity();
        j++;
      }
    }

    j *= 3;
    size_t k = 0;
    for(size_t i =  info_.numThreeDofContacts; 
      i < info_.numThreeDofContacts +  info_.numSixDofContacts; ++i)
    {
      if(contactFlags_[i])
      {
        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;
        a.block(6 * k + j, info_.generalizedCoordinatesNum + startIndex, 6, 6) = matrix6_t::Identity();
        k++;
      }
    }
    
    b = inputDesired_.head(a.rows());

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WbcBase::Settings loadWbcBaseSettings(const std::string& filename,
    const std::string& fieldName, bool verbose)
  {
    WbcBase::Settings settings;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged Whole Body Controller Base Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    auto& endEffectorThreeDofNames = settings.endEffectorThreeDofNames;
    auto& endEffectorSixDofNames = settings.endEffectorSixDofNames;

    loadData::loadPtreeValue(pt, settings.desiredFrequency, fieldName + ".desiredFrequency", verbose);
    if(settings.desiredFrequency < 0.0)
    {
      throw std::invalid_argument("[WbcBase]: desired frequency smaller than 0!");
    }

    loadData::loadPtreeValue(pt, settings.worldLinkName, fieldName + ".worldLinkName", verbose);

    loadData::loadPtreeValue(pt, settings.baseLinkName, fieldName + ".baseLinkName", verbose);

    loadData::loadStdVector(filename, fieldName + ".endEffectorThreeDofNames", endEffectorThreeDofNames, verbose);

    loadData::loadStdVector(filename, fieldName + ".endEffectorSixDofNames", endEffectorSixDofNames, verbose);

    std::unordered_set<std::string> namesSet;

    std::vector<std::string> namesVector = endEffectorThreeDofNames;
    namesVector.insert(namesVector.end(), 
        endEffectorSixDofNames.begin(), endEffectorSixDofNames.end());

    for(const auto& endEffectorName: namesVector)
    {
      if(namesSet.find(endEffectorName) == namesSet.cend())
      {
        namesSet.emplace(endEffectorName);
      }
      else
      {
        std::string message = "[ModelSettings]: End effector " +  endEffectorName + " already used!";
        throw std::invalid_argument(message);
      }
    }

    loadData::loadPtreeValue(pt, settings.useDynamicsTask, fieldName + ".useDynamicsTask", verbose);
    loadData::loadPtreeValue(pt, settings.useEndEffectorsTrackingTask, fieldName + ".useEndEffectorsTrackingTask", verbose);
    loadData::loadPtreeValue(pt, settings.useContactForceTrackingTask, fieldName + ".useContactForceTrackingTask", verbose);
    loadData::loadPtreeValue(pt, settings.useTorqueLimitsTask, fieldName + ".useTorqueLimitsTask", verbose);
    loadData::loadPtreeValue(pt, settings.useKinematicContactTask, fieldName + ".useKinematicContactTask", verbose);
    loadData::loadPtreeValue(pt, settings.useFrictionConeTask, fieldName + ".useFrictionConeTask", verbose);
    loadData::loadPtreeValue(pt, settings.useBaseTrackingTask, fieldName + ".useBaseTrackingTask", verbose);
  
    if(settings.useBaseTrackingTask)
    {
      loadData::loadEigenMatrix(filename, fieldName + ".baseSettings" + ".linearFeedForwardGain", settings.baseSettings.linearFeedForwardGain);
      if((settings.baseSettings.linearFeedForwardGain.array() < 0.0).any())
      {
        throw std::invalid_argument("[WbcBase]: Base linear feedforward gain weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".baseSettings" + ".linearProportionalGain", settings.baseSettings.linearProportionalGain);
      if((settings.baseSettings.linearProportionalGain.array() < 0.0).any())
      {
        throw std::invalid_argument("[WbcBase]: Base linear proportional gain weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".baseSettings" + ".linearDerivativeGain", settings.baseSettings.linearDerivativeGain);
      if((settings.baseSettings.linearDerivativeGain.array() < 0.0).any())
      {
        throw std::invalid_argument("[WbcBase]: Base linear derivative gain weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".baseSettings" + ".angularFeedForwardGain", settings.baseSettings.angularFeedForwardGain);
      if((settings.baseSettings.angularFeedForwardGain.array() < 0.0).any())
      {
        throw std::invalid_argument("[WbcBase]: Base angular feedforward gain weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".baseSettings" + ".angularProportionalGain", settings.baseSettings.angularProportionalGain);
      if((settings.baseSettings.angularProportionalGain.array() < 0.0).any())
      {
        throw std::invalid_argument("[WbcBase]: Base angular proportional gain weights vector element smaller than 0!");
      }

      loadData::loadEigenMatrix(filename, fieldName + ".baseSettings" + ".angularDerivativeGain", settings.baseSettings.angularDerivativeGain);
      if((settings.baseSettings.angularDerivativeGain.array() < 0.0).any())
      {
        throw std::invalid_argument("[WbcBase]: Base angular derivative gain weights vector element smaller than 0!");
      }
    }

    if(settings.useFrictionConeTask)
    {
      loadData::loadPtreeValue(pt, settings.frictionConeSettings.frictionCoefficient, fieldName + ".frictionConeSettings.frictionCoefficient", verbose);
      if(settings.frictionConeSettings.frictionCoefficient < 0.0)
      {
        throw std::invalid_argument("[WbcBase]: Fricition coefficient smaller than 0!");
      }
    }

    if(settings.useEndEffectorsTrackingTask)
    {
      auto& endEffectorWeights = settings.endEffectorSettings;
      
      const size_t endEffectorNum = endEffectorThreeDofNames.size() 
        + endEffectorSixDofNames.size();

      endEffectorWeights.linearFeedForwardGain.resize(endEffectorNum);
      endEffectorWeights.linearProportionalGain.resize(endEffectorNum);
      endEffectorWeights.linearDerivativeGain.resize(endEffectorNum);
      endEffectorWeights.angularFeedForwardGain.resize(endEffectorSixDofNames.size());
      endEffectorWeights.angularProportionalGain.resize(endEffectorSixDofNames.size());
      endEffectorWeights.angularDerivativeGain.resize(endEffectorSixDofNames.size());

      std::vector<std::reference_wrapper<const std::string>> endEffectorNames;

      for(const auto& threeDofContact: endEffectorThreeDofNames)
      {
        endEffectorNames.push_back(threeDofContact);
      }

      for(const auto& sixDofContact: endEffectorSixDofNames)
      {
        endEffectorNames.push_back(sixDofContact);
      }

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        const std::string& name = endEffectorNames[i].get();
        auto& linearFF = endEffectorWeights.linearFeedForwardGain[i];
        auto& linearKp = endEffectorWeights.linearProportionalGain[i];
        auto& linearKd = endEffectorWeights.linearDerivativeGain[i];

        loadData::loadEigenMatrix(filename, fieldName + ".endEffectorSettings" + "." + name + ".linearFeedForwardGain", linearFF);
        if((linearFF.array() < 0.0).any())
        {
          std::string message = "[WbcBase]: " + name + " feedforward weights vector element smaller than 0!";
          throw std::invalid_argument(message);
        }

        loadData::loadEigenMatrix(filename, fieldName + ".endEffectorSettings" + "." + name + ".linearProportionalGain", linearKp);
        if((linearKp.array() < 0.0).any())
        {
          std::string message = "[WbcBase]: " + name + " linear proportional weights vector element smaller than 0!";
          throw std::invalid_argument(message);
        }

        loadData::loadEigenMatrix(filename, fieldName + ".endEffectorSettings" + "." + name + ".linearDerivativeGain", linearKd);
        if((linearKd.array() < 0.0).any())
        {
          std::string message = "[WbcBase]: " + name + " linear derivative weights vector element smaller than 0!";
          throw std::invalid_argument(message);
        }

        // Only for 6 DoF end effectors
        if(i >= endEffectorThreeDofNames.size())
        {
          auto& angularFF = endEffectorWeights.angularFeedForwardGain[i - endEffectorThreeDofNames.size()];
          auto& angularKp = endEffectorWeights.angularProportionalGain[i - endEffectorThreeDofNames.size()];
          auto& angularKd = endEffectorWeights.angularDerivativeGain[i - endEffectorThreeDofNames.size()];

          loadData::loadEigenMatrix(filename, fieldName + ".endEffectorSettings" + "." + name + ".angularFeedForwardGain", angularFF);
          if((angularFF.array() < 0.0).any())
          {
            std::string message = "[WbcBase]: " + name + " feedforward weights vector element smaller than 0!";
            throw std::invalid_argument(message);
          }

          loadData::loadEigenMatrix(filename, fieldName + ".endEffectorSettings" + "." + name + ".angularProportionalGain", angularKp);
          if((angularKp.array() < 0.0).any())
          {
            std::string message = "[WbcBase]: " + name + " angular proportional weights vector element smaller than 0!";
            throw std::invalid_argument(message);
          }

          loadData::loadEigenMatrix(filename, fieldName + ".endEffectorSettings" + "." + name + ".angularDerivativeGain", angularKd);
          if((angularKd.array() < 0.0).any())
          {
            std::string message = "[WbcBase]: " + name + " angular derivative weights vector element smaller than 0!";
            throw std::invalid_argument(message);
          }
        }
      }
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_whole_body_control 