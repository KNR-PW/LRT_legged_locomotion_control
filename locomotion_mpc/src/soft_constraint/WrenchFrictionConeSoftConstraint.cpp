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

#include <legged_locomotion_mpc/soft_constraint/WrenchFrictionConeSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeSoftConstraint::WrenchFrictionConeSoftConstraint(
    const LeggedReferenceManager &referenceManager,
    Config config,
    FloatingBaseModelInfo info): 
      StateInputCost(),
      referenceManager_(referenceManager),
      config_(config),
      info_(std::move(info)), 
      frictionBarrierPenaltyPtr_(new RelaxedBarrierPenalty(config.barrierSettings))

  {
    if(config_.frictionCoefficient < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Fricition coefficient smaller than 0!");
    }

    if(config_.footHalfLengthX < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Foot length in direction X smaller than 0!");
    }

    if(config_.footHalfLengthY < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Foot length in direction Y smaller than 0!");
    }

    coneConstraintMatrix_ = generateConeConstraintMatrix(config_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeSoftConstraint* WrenchFrictionConeSoftConstraint::clone() const
  { 
    return new WrenchFrictionConeSoftConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeSoftConstraint::WrenchFrictionConeSoftConstraint(
    const WrenchFrictionConeSoftConstraint &other):
      referenceManager_(other.referenceManager_), config_(other.config_),
      info_(other.info_), coneConstraintMatrix_(other.coneConstraintMatrix_), 
      frictionBarrierPenaltyPtr_(other.frictionBarrierPenaltyPtr_->clone()) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t WrenchFrictionConeSoftConstraint::getValue(scalar_t time, const vector_t& state, 
    const vector_t& input, const TargetTrajectories& targetTrajectories, 
    const PreComputation& preComp) const 
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;

    scalar_t cost = 0.0;

    for(size_t i = info_.numThreeDofContacts; i < endEffectorNum; ++i)
    {
      if(!contactFlags[i]) continue;

      const auto wrenchInWorldFrame = access_helper_functions::getContactWrenches(input, i, 
        info_);

      // Get rotation matrix to foot frame because the axes are consistent with the leg lengths
      const vector3_t eulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrixToTerrain = getRotationMatrixFromZyxEulerAngles(
        eulerAngles).transpose();
      
      vector6_t localWrench;
      localWrench.block<3, 1>(0, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(0, 0);
      localWrench.block<3, 1>(3, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(3, 0);

      const Eigen::Vector<scalar_t, 16> coneConstraint = coneConstraintMatrix_ * localWrench;

      cost += coneConstraint.unaryExpr([&](scalar_t hi) 
        {
          return frictionBarrierPenaltyPtr_->getValue(0.0, hi);
        }).sum();
    }
    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ScalarFunctionQuadraticApproximation WrenchFrictionConeSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, 
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const 
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdu = vector_t::Zero(info_.inputDim);
    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
    cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);
    cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);

    const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;

    for(size_t i = info_.numThreeDofContacts; i < endEffectorNum; ++i)
    {
      if(!contactFlags[i]) continue;
      
      const auto wrenchInWorldFrame = access_helper_functions::getContactWrenches(input, i, 
        info_);

      // Get rotation matrix to foot frame because the axes are consistent with the leg lengths
      const vector3_t eulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrixToTerrain = getRotationMatrixFromZyxEulerAngles(
        eulerAngles).transpose();

      vector6_t localWrench;
      localWrench.block<3, 1>(0, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(0, 0);
      localWrench.block<3, 1>(3, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(3, 0);

      const Eigen::Vector<scalar_t, 16> coneConstraint = coneConstraintMatrix_ * localWrench;

      cost.f += coneConstraint.unaryExpr([&](scalar_t hi) 
        {
          return frictionBarrierPenaltyPtr_->getValue(0.0, hi);
        }).sum();

      const Eigen::Vector<scalar_t, 16> barrierPenaltyDerivative = coneConstraint.unaryExpr([&](scalar_t hi) 
        {
          return frictionBarrierPenaltyPtr_->getDerivative(0.0, hi);
        });

      const Eigen::Vector<scalar_t, 16> barrierPenaltySecondDerivative = coneConstraint.unaryExpr([&](scalar_t hi) 
        {
          return frictionBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
        });

      matrix6_t rotation6x6 = matrix6_t::Zero();
      rotation6x6.block<3, 3>(0, 0) = rotationMatrixToTerrain;
      rotation6x6.block<3, 3>(3, 3) = rotationMatrixToTerrain;

      const Eigen::Matrix<ocs2::scalar_t, 16, 6> rotatedConeConstraintMatrix = 
        coneConstraintMatrix_ * rotation6x6;
      
      const size_t startIndex = (3 * info_.numThreeDofContacts + 6 * (i - info_.numThreeDofContacts));
      
      cost.dfdu.block<6, 1>(startIndex, 0) += rotatedConeConstraintMatrix.transpose() 
        * barrierPenaltyDerivative;
  
      cost.dfduu.block<6, 6>(startIndex, startIndex) += rotatedConeConstraintMatrix.transpose() 
        * barrierPenaltySecondDerivative.asDiagonal() * rotatedConeConstraintMatrix;
    }
    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Matrix<scalar_t, 16, 6> WrenchFrictionConeSoftConstraint::generateConeConstraintMatrix(
    const Config& config)
  {
    Eigen::Matrix<scalar_t, 16, 6> coneConstraintMatrix = Eigen::Matrix<scalar_t, 16, 6>::Zero();

    // u * f_z + f_x >= 0
    coneConstraintMatrix(0, 0) = 1.0;
    coneConstraintMatrix(0, 2) = config.frictionCoefficient;

    // u * f_z - f_x >= 0
    coneConstraintMatrix(1, 0) = -1.0;
    coneConstraintMatrix(1, 2) = config.frictionCoefficient;

    // u * f_z + f_y >= 0
    coneConstraintMatrix(2, 1) = 1.0;
    coneConstraintMatrix(2, 2) = config.frictionCoefficient;

    // u * f_z - f_y >= 0
    coneConstraintMatrix(3, 1) = -1.0;
    coneConstraintMatrix(3, 2) = config.frictionCoefficient;

    // Y * f_z + tau_x >= 0
    coneConstraintMatrix(4, 2) = config.footHalfLengthY;
    coneConstraintMatrix(4, 3) = 1.0;

    // Y * f_z - tau_x >= 0
    coneConstraintMatrix(5, 2) = config.footHalfLengthY;
    coneConstraintMatrix(5, 3) = -1.0;

    // X * f_z + tau_y >= 0
    coneConstraintMatrix(6, 2) = config.footHalfLengthX;
    coneConstraintMatrix(6, 4) = 1.0;

    // X * f_z - tau_y >= 0
    coneConstraintMatrix(7, 2) = config.footHalfLengthX;
    coneConstraintMatrix(7, 4) = -1.0;

    // Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y + tau_z >= 0
    coneConstraintMatrix(8, 0) = config.footHalfLengthY;
    coneConstraintMatrix(8, 1) = config.footHalfLengthX;
    coneConstraintMatrix(8, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(8, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(8, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(8, 5) = 1;

    // Y * f_x - X * f_y + u * (X + Y) * f_z - u * tau_x + u * tau_y + tau_z >= 0
    coneConstraintMatrix(9, 0) = config.footHalfLengthY;
    coneConstraintMatrix(9, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(9, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(9, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(9, 4) = config.frictionCoefficient;
    coneConstraintMatrix(9, 5) = 1;

    // -Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y + tau_z >= 0
    coneConstraintMatrix(10, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(10, 1) = config.footHalfLengthX;
    coneConstraintMatrix(10, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(10, 3) = config.frictionCoefficient;
    coneConstraintMatrix(10, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(10, 5) = 1;

    // -Y * f_x - X * f_y + u * (X + Y) * f_z + u * tau_x + u * tau_y + tau_z >= 0
    coneConstraintMatrix(11, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(11, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(11, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(11, 3) = config.frictionCoefficient;
    coneConstraintMatrix(11, 4) = config.frictionCoefficient;
    coneConstraintMatrix(11, 5) = 1;

    // -Y * f_x - X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y - tau_z >= 0
    coneConstraintMatrix(12, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(12, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(12, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(12, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(12, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(12, 5) = -1;

    // -Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x + u * tau_y - tau_z >= 0
    coneConstraintMatrix(13, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(13, 1) = config.footHalfLengthX;
    coneConstraintMatrix(13, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(13, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(13, 4) = config.frictionCoefficient;
    coneConstraintMatrix(13, 5) = -1;

    // Y * f_x - X * f_y + u * (X + Y) * f_z + u * tau_x - u * tau_y - tau_z >= 0
    coneConstraintMatrix(14, 0) = config.footHalfLengthY;
    coneConstraintMatrix(14, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(14, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(14, 3) = config.frictionCoefficient;
    coneConstraintMatrix(14, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(14, 5) = -1;

    // Y * f_x + X * f_y + u * (X + Y) * f_z + u * tau_x + u * tau_y - tau_z >= 0
    coneConstraintMatrix(15, 0) = config.footHalfLengthY;
    coneConstraintMatrix(15, 1) = config.footHalfLengthX;
    coneConstraintMatrix(15, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(15, 3) = config.frictionCoefficient;
    coneConstraintMatrix(15, 4) = config.frictionCoefficient;
    coneConstraintMatrix(15, 5) = -1;

    return coneConstraintMatrix;
  }

  WrenchFrictionConeSoftConstraint::Config loadWrenchFrictionConeConfig(
    const std::string& filename, const std::string& fieldName,bool verbose)
  {
    boost::property_tree::ptree pt;
    read_info(filename, pt);

    WrenchFrictionConeSoftConstraint::Config config;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Wrench Friction Cone Constraint Config:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, config.frictionCoefficient, fieldName + ".frictionCoefficient", verbose);

    if(config.frictionCoefficient < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Fricition coefficient smaller than 0!");
    }

    loadData::loadPtreeValue(pt, config.footHalfLengthX, fieldName + ".footLengthX", verbose);
    config.footHalfLengthX *= 0.5;

    if(config.footHalfLengthX < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Foot length in direction X smaller than 0!");
    }
    
    loadData::loadPtreeValue(pt, config.footHalfLengthY, fieldName + ".footLengthY", verbose);
    config.footHalfLengthY *= 0.5;

    if(config.footHalfLengthY < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Foot length in direction Y smaller than 0!");
    }

    loadData::loadPtreeValue(pt, config.barrierSettings.mu, 
      fieldName + ".mu", verbose);

    if(config.barrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, config.barrierSettings.delta, 
      fieldName + ".delta", verbose);

    if(config.barrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[WrenchFrictionConeSoftConstraint]: Relaxed barrier penalty delta smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }
      
    return config;
  }

} // namespace legged_locomotion_mpc