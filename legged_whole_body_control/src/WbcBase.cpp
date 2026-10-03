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

#include <legged_whole_body_control/WbcBase.h>

#include <utility>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.h>
#include <floating_base_model/ModelHelperFunctions.h>

namespace legged_whole_body_control  
{
  using namespace ocs2;
  using namespace floating_base_model;

  WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, 
    FloatingBaseModelInfo info, WbcBase::Settings settings):
      settings_(std::move(settings)), info_(info), mapping_(info)
      pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
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
  }

  void WbcBase:updateDesired(scalar_t time, const vector_t& state, 
    const vector_t& input)
  {
    previousTimeDesired_ = timeDesired_;
    timeDesired_ = time;

    previousStateDesired_ = std::move(stateDesired_);
    stateDesired_ = state;

    previousInputDesired_ = std::move(inputDesired_);
    inputDesired_ = input;
  }

  void WbcBase::updateCurrent(scalar_t time, const vector_t& state, 
    const vector_t& input)
  {
    timeMeasured_ = time;
    stateMeasured_ = state;
    inputMeasured_ = input;
  }

  void WbcBase::updateContactFlags(scalar_t time, 
    const contact_flags_t& contactFlags)
  {
    timeContact_ = time;
    contactFlags_ = contactFlags;
  }

  vector6_t& WbcBase::getBaseAcceleration() const
  {
    return currentResult_.head<6>().finished();
  }

  vector_t& WbcBase::getJointAcceleration() const
  {
    return currentResult_.block(6, 0, info_.actuatedDofNum, 1).finished();
  }

  vector_t& WbcBase::getJointTorque() const;
  {
    return currentResult_.tail(info_.actuatedDofNum).finished();
  }

  vector3_t& WbcBase::getEndEffectorForce(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex >= 0);
    assert(endEffectorIndex < info_.numThreeDofContacts);
    size_t contactStartIndex = info_.generalizedCoordinatesNum + 3 * endEffectorIndex;
    return currentResult_.block<3, 1>(contactStartIndex, 0).finished();
  }

  vector6_t& WbcBase::getEndEffectorWrench(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex >= info_.numThreeDofContacts);
    assert(endEffectorIndex < info_.numThreeDofContacts + info_.numSixDofContacts);
    size_t contactStartIndex = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + 6 * (endEffectorIndex - info_.numThreeDofContacts);
    return currentResult_.block<6, 1>(contactStartIndex, 0).finished();
  }

  void WbcBase::calculate(scalar_t time)
  {
    calculateMeasured();
    calculateDesired();
  }

  void WbcBase::calculateMeasured() 
  {
    mapping_.setPinocchioInterface(pinocchioInterfaceMeasured_);

    const vector_t generalizedPositions = mapping_.getPinocchioJointPosition(stateMeasured_);
    const vector_t generalizedVelocities = mapping_.getPinocchioJointVelocity(stateMeasured_, 
      inputMeasured_);

    const auto& model = pinocchioInterfaceMeasured_.getModel();
    auto& data = pinocchioInterfaceMeasured_.getData();

    pinocchio::forwardKinematics(model, data, generalizedPositions, generalizedVelocities);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured_);
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

    pinocchio::computeJointJacobiansTimeVariation(model, data, generalizedPositions, generalizedVelocitie);

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

  void WbcBase::calculateDesired() 
  {
    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

    const auto& model = pinocchioInterfaceDesired_.getModel();
    auto& data = pinocchioInterfaceDesired_.getData();

    const vector_t generalizedPosition = mapping_.getPinocchioJointPosition(stateDesired_);
    const vector_t currentVelocities = mapping_.getPinocchioJointVelocity(stateDesired_, inputDesired_);
    const vector_t previousVelocities = mapping_.getPinocchioJointVelocity(
      previousStateDesired_, previousInputDesired_);

    const vector_t generalizedAcceleration = (currentVelocities - previousVelocities) / (timeDesired_ - previousTimeDesired_);

    feedFrowardBaseAcceleration_ = generalizedAcceleration.block<6, 1>(0, 0);

    pinocchio::forwardKinematics(model, data, generalizedPosition, currentVelocities, 
      generalizedAcceleration);
    pinocchio::computeJointJacobians(model, data, generalizedPosition);
    pinocchio::updateFramePlacements(model, data);
  }

  Task WbcBase::formulateDynamicsTask() const
  {
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.block(0, 0, info_.actuatedDofNum, 6).setZero();
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numberOfDecisionVariables_) 
      << data.M, -stackedJacobians_.transpose(), -s.transpose()).finished();
    vector_t b = -data.nle;

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  Task WbcBase::formulateTorqueLimitsTask() const
  {
    matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

    matrix_t d(2 * info_.actuatedDofNum, numberOfDecisionVariables_);
    d.setZero();
    const size_t startIndex = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
    d.block(0, startIndex, info_.actuatedDofNum, info_.actuatedDofNum) = i;
    d.block(info_.actuatedDofNum, startIndex, info_.actuatedDofNum) = -i;

    const auto& model = pinocchioInterfaceDesired_.getModel();
    const vector_t jointMaxTorque =  model.effortLimit.segment(6, 
      floatingBaseModelInfo_.actuatedDofNum);

    vector_t f(2 * info_.actuatedDofNum);
    f.segment(0, info_.actuatedDofNum) = jointMaxTorque;
    f.segment(info_.actuatedDofNum, info_.actuatedDofNum) = jointMaxTorque;
    
    return Task(matrix_t(), vector_t(), std::move(d), std::move(f));
  }

  Task WbcBase::formulateKinematicContactTask() const
  {
    mapping_.setPinocchioInterface(pinocchioInterfaceMeasured_);

    const vector_t generalizedVelocities = mapping_.getPinocchioJointVelocity(stateMeasured_, 
      inputMeasured_);

    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    matrix_t a(3 * numberOf3DofContacts + 6 * numberOf6DofContacts, 
      numberOfDecisionVariables_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) 
    {
      if(contactFlags_[i]) 
      {
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = stackedJacobians_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = -stackedJacobianDerivatives_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * generalizedVelocities;
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
        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContact;
        a.block(6 * k + j, 0, 3, info_.generalizedCoordinatesNum) = stackedJacobians_.block(startIndex, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(6 * k + j, 3) = -stackedJacobianDerivatives_.block(startIndex, 0, 3, info_.generalizedCoordinatesNum) * generalizedVelocities;
        k++;
      }
    }

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  Task WbcBase::formulateFrictionConeTask() const
  {
    // No contact == zero force/wrench
    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    matrix_t a(3 * (info_.numThreeDofContacts - numberOf3DofContacts) 
      + 6 * (info_.numSixDofContacts - numberOf6DofContacts), 
      numberOfDecisionVariables_);
    a.setZero();

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if(!contactFlags_[i]) 
      {
        a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      }
    }

    for (size_t i = info_.numThreeDofContacts; 
      i < info_.numThreeDofContacts + info_.numSixDofContacts; i++)
    {
      if(!contactFlags_[i]) 
      {
        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContact;
        a.block(startIndex, info_.generalizedCoordinatesNum + startIndex, 6, 6) = matrix_t::Identity(6, 6);
      }
    }

    vector_t b(a.rows());
    b.setZero();

    // Calculate heading and lateral vectors:
    const vector3_t baseEulerAngles = access_helper_functions::getBaseOrientationZyx(
      stateMeasured_, info_);

    const matrix3_t baseRotationMatrix = getRotationMatrixFromZyxEulerAngles(baseEulerAngles);

    const vector3_t baseHeading = baseRotationMatrix * vector3_t(
      access_helper_functions::getBaseLinearVelocity(stateMeasured_, info_)).normalized();

    matrix_t d(5 * (numberOf63ofContacts + numberOf6DofContacts), numberOfDecisionVariables_);
    d.setZero();

    size_t j = 0;
    for(size_t i = 0; i < info_.numThreeDofContacts + info_.numSixDofContacts; ++i)
    {
      if(contactFlags_[i]) 
      {
        const auto& normal = terrainNormals_[i];
        const vector3_t heading = (baseHeading - normal.dot(baseHeading) * normal).normalized();
        const vector3_t lateral = normal.cross(heading);
        const vector3_t frictionNormal = settings_.frictionConeSettings.frictionCoefficient * normal;
        
        Eigen::Matrix<scalar_t, 5, 3> frictionPyramic;
        frictionPyramic << -normal.transpose(), 
                           (heading - frictionNormal).transpose(),
                           -(heading + frictionNormal).transpose(),
                           (lateral - frictionNormal).transpose(),
                           -(lateral + frictionNormal).transpose();

        const size_t offset = [i, info_.numThreeDofContacts]()
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

  Task WbcBase::formulateBaseTrackingTask() const
  {
    matrix_t a(6, numberOfDecisionVariables_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

    const auto& baseSettings = settings_.baseSettings;

    const auto& basePositionMeasured = getBasePosition(stateMeasured_, info_);
    const auto& basePositionDesired = getBasePosition(stateDesired_, info_);

    const auto& baseEulerMeasured = getBaseOrientationZyx(stateMeasured_, info_);
    const auto& baseEulerDesired = getBaseOrientationZyx(stateDesired_, info_);

    const auto targetRotation = getRotationMatrixFromZyxEulerAngles(baseEulerDesired);
    const auto currentRotation = getRotationMatrixFromZyxEulerAngles(baseEulerMeasured);

    const auto& baseLinearVelocityMeasured = getBaseLinearVelocity(stateMeasured_, info_);
    const auto& baseLinearVelocityDesired = getBaseLinearVelocity(stateDesired_, info_);

    const auto& baseAngularVelocityMeasured = getBaseAngularVelocity(stateMeasured_, info_);
    const auto& baseAngularVelocityDesired = getBaseAngularVelocity(stateDesired_, info_);

    const vector3_t basePositionError = currentRotation.transpose() * (
      basePositionDesired - basePositionMeasured);
    const vector3_t baseOrientationError = pinocchio::log3(targetRotation.transpose() * currentRotation);
    const vector3_t baseLinearVelocityError = baseLinearVelocityDesired - baseLinearVelocityMeasured;
    const vector3_t baseAngularVelocityError = baseAngularVelocityDesired - baseAngularVelocityMeasured;

    vector6_t b;
    b.segment<3>(0) = baseSettings.linearFeedForwardGain.asDiagonal() * feedFrowardBaseAcceleration_.segment<3>(0);
    b.segment<3>(0) += baseSettings.linearProportionalGain.asDiagonal() * basePositionError;
    b.segment<3>(0) += baseSettings.linearDerivativeGain.asDiagonal() * baseLinearVelocityError;

    b.segment<3>(3) = baseSettings.linearFeedForwardGain.asDiagonal() * feedFrowardBaseAcceleration_.segment<3>(3);
    b.segment<3>(3) += baseSettings.angularProportionalGain.asDiagonal() * baseOrientationErrorError;
    b.segment<3>(3) += baseSettings.angularDerivativeGain.asDiagonal() * baseAngularVelocityError;
    
    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  Task WbcBase::formulateEndEffectorsTrackingTask() const
  {
    mapping_.setPinocchioInterface(pinocchioInterfaceMeasured_);

    const vector_t generalizedVelocities = mapping_.getPinocchioJointVelocity(
      stateMeasured_, inputMeasured_);

    const size_t numberOf6DofContacts = (contactFlags_ >> info_.numThreeDofContacts).count();
    const size_t numberOf3DofContacts = contactFlags_.count() - numberOf6DofContacts;

    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

    const auto& modelMeasured = pinocchioInterfaceMeasured_.getModel();
    auto& dataMeasured = pinocchioInterfaceMeasured_.getData();

    const auto& modelDesired = pinocchioInterfaceDesired _.getModel();
    auto& dataDesired  = pinocchioInterfaceDesired _.getData();

    const auto& endEffectorSettings = settings_.endEffectorSettings;

    const auto

    matrix_t a(3 * (info_.numThreeDofContacts - numberOf3DofContacts) 
      + 6 * (info_.numSixDofContacts - numberOf6DofContacts), numberOfDecisionVariables_);
    a.setZero();

    vector_t b(a.rows());
    b.setZero();

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
        acceleration += kp * (positionDesired - positionMeasured);
        acceleration += kd * (velocityDesired - velocityMeasured);
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
          dataDesired, frameId, rf).asVector();

        const auto& positionMeasured = dataMeasured.oMf[frameId].translation();
        const auto& rotationMeasured = dataMeasured.oMf[frameId].rotation();
        const vector6_t velocityMeasured = pinocchio::getFrameVelocity(modelMeasured, 
          dataMeasured, frameId, rf).asVector();
        
        const vector3_t orientationError = rotationMeasured * pinocchio::log3(rotationDesired.transpose() * rotationMeasured);
        const vector6_t velocityError = velocityDesired - velocityMeasured;

        const vector6_t accelerationFeedForward = getFrameClassicalAcceleration(
          modelDesired, dataDesired, frameId, rf).asVector();

        vector6_t acceleration; 

        acceleration.segment<3>(0) =  linearFf.asDiagonal() * accelerationFeedForward.segment<3>(0);
        acceleration.segment<3>(0) += linearKp * (positionDesired - positionMeasured);
        acceleration.segment<3>(0) += linearKd * velocityError.segment<3>(0);

        acceleration.segment<3>(3) =  angularFf.asDiagonal() * accelerationFeedForward.segment<3>(3);
        acceleration.segment<3>(3) += angularKp * orientationError;
        acceleration.segment<3>(3) += angularKd * velocityError.segment<3>(3);

        const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;

        a.block(6 * k + j, 0, 6, info_.generalizedCoordinatesNum) = stackedJacobians_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum);
        b.segment(6 * k + j, 6) = acceleration - stackedJacobianDerivatives_.block(startIndex, 0, 6, info_.generalizedCoordinatesNum) * generalizedVelocities;
        k++;
      }
    }

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  Task WbcBase::formulateContactForceTrackingTask() const 
  {
    matrix_t a(3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts, 
      numberOfDecisionVariables_);
    a.setZero();

    vector_t b(a.rows());

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
    {
      a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }

    for(size_t i =  info_.numThreeDofContacts; 
      i < info_.numThreeDofContacts +  info_.numSixDofContacts; ++i)
    {
      const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;
      a.block(startIndex, info_.generalizedCoordinatesNum + startIndex, 6, 6) = matrix_t::Identity(6, 6);
    }
    
    b = inputDesired.head(a.rows());

    return Task(std::move(a), std::move(b), matrix_t(), vector_t());
  }

  void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
    // Load task file
    torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
    loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
    if (verbose) {
      std::cerr << "\n #### Torque Limits Task:";
      std::cerr << "\n #### =============================================================================\n";
      std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
      std::cerr << " #### =============================================================================\n";
    }
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "frictionConeTask.";
    if (verbose) {
      std::cerr << "\n #### Friction Cone Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
    if (verbose) {
      std::cerr << " #### =============================================================================\n";
    }
    prefix = "swingLegTask.";
    if (verbose) {
      std::cerr << "\n #### Swing Leg Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
  }
} // namespace legged_whole_body_control 
