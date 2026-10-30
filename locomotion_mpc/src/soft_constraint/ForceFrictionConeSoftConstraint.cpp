/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2025

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <legged_locomotion_mpc/soft_constraint/ForceFrictionConeSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeSoftConstraint::ForceFrictionConeSoftConstraint(
    const LeggedReferenceManager &referenceManager, Config config,
    FloatingBaseModelInfo info): 
      StateInputCost(),
      referenceManager_(referenceManager),
      config_(config),
      info_(std::move(info)), 
      frictionBarrierPenaltyPtr_(new RelaxedBarrierPenalty(config.barrierSettings))
  {
    if(config_.frictionCoefficient < 0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Friction coefficient smaller than 0!");
    }

    if(config_.regularization < 0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Regularization smaller than 0!");
    }
    
    if(config_.hessianDiagonalShift < 0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Hessian diagonal shift smaller than 0!");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeSoftConstraint* ForceFrictionConeSoftConstraint::clone() const
  { 
    return new ForceFrictionConeSoftConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t ForceFrictionConeSoftConstraint::getValue(scalar_t time, const vector_t& state,
    const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const 
  {

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    scalar_t cost = 0.0;

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    for(size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if(!contactFlags[i]) continue;

      const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, 
      i, info_);

      const matrix3_t& rotationMatrixToTerrain = 
      leggedPrecomputation.getRotationWorldToTerrain(i);

      const vector3_t localForce = rotationMatrixToTerrain * forcesInWorldFrame;

      cost += frictionBarrierPenaltyPtr_->getValue(0.0, coneConstraint(localForce));
    }

    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ScalarFunctionQuadraticApproximation ForceFrictionConeSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const 
  {

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdu = vector_t::Zero(info_.inputDim);
    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
    cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);
    cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    for(size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if(!contactFlags[i]) continue;

      const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, 
        i, info_);

      const matrix3_t& rotationMatrixToTerrain = 
        leggedPrecomputation.getRotationWorldToTerrain(i);

      const vector3_t localForce = rotationMatrixToTerrain * forcesInWorldFrame;

      const scalar_t forceXSquared = localForce.x() * localForce.x();
      const scalar_t forceYSquared = localForce.y() * localForce.y();
      const scalar_t forceXforceY  = localForce.x() * localForce.y();

      const scalar_t tangentForceSquaredInverse = 1 / (forceXSquared + 
        forceYSquared + config_.regularization);

      const scalar_t tangentForceInverse = sqrt(tangentForceSquaredInverse);
      
      const scalar_t tangentForceInverse32 = tangentForceSquaredInverse * tangentForceInverse;

      vector3_t dConeDF;
      dConeDF(0) = -localForce.x() * tangentForceInverse;
      dConeDF(1) = -localForce.y() * tangentForceInverse;
      dConeDF(2) = config_.frictionCoefficient;

      
      matrix3_t d2ConeDF2 = matrix3_t::Zero();

      d2ConeDF2(0, 0) = -(forceYSquared + config_.regularization) * tangentForceInverse32;
      d2ConeDF2(1, 1) = -(forceXSquared + config_.regularization) * tangentForceInverse32;
      d2ConeDF2(0, 1) =  forceXforceY * tangentForceInverse32;
      d2ConeDF2(1, 0) =  d2ConeDF2(0, 1);

      const scalar_t constraintValue = coneConstraint(localForce);

      cost.f += frictionBarrierPenaltyPtr_->getValue(0.0, constraintValue);

      const scalar_t penaltyDerivative = frictionBarrierPenaltyPtr_->getDerivative(0.0, 
        constraintValue);

      const vector3_t dConedU = rotationMatrixToTerrain.transpose() * dConeDF;

      cost.dfdu.block<3, 1>(3 * i, 0) += penaltyDerivative * dConedU;

      const scalar_t penaltySecondDerivative = frictionBarrierPenaltyPtr_->getSecondDerivative(0.0, 
        constraintValue);  

      matrix3_t d2ConeD2U = rotationMatrixToTerrain.transpose() * d2ConeDF2
        * rotationMatrixToTerrain;

      // Cone is concave, shift makes its force Hessian negative definite (as in ocs2)
      d2ConeD2U.diagonal().array() -= config_.hessianDiagonalShift;

      cost.dfduu.block<3, 3>(3 * i, 3 * i) = dConedU * penaltySecondDerivative * dConedU.transpose() 
        + penaltyDerivative * d2ConeD2U;
    }

    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t ForceFrictionConeSoftConstraint::coneConstraint(const vector3_t &localForce) const 
  {
    const scalar_t tangentForce = sqrt(localForce.x() * localForce.x() + 
      localForce.y() * localForce.y() + config_.regularization);

    const scalar_t frictionForce = config_.frictionCoefficient * localForce.z();
      
    const scalar_t coneConstraint = frictionForce - tangentForce;
    return coneConstraint;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeSoftConstraint::ForceFrictionConeSoftConstraint(
    const ForceFrictionConeSoftConstraint &other): StateInputCost(),
      referenceManager_(other.referenceManager_), config_(other.config_), info_(other.info_), 
      frictionBarrierPenaltyPtr_(other.frictionBarrierPenaltyPtr_->clone()) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeSoftConstraint::Config loadForceFrictionConeConfig(
    const std::string& filename, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    read_info(filename, pt);

    ForceFrictionConeSoftConstraint::Config config;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Force Friction Cone Soft Constraint Config:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, config.frictionCoefficient, fieldName + ".frictionCoefficient", verbose);

    if(config.frictionCoefficient < 0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Friction coefficient smaller than 0!");
    }

    loadData::loadPtreeValue(pt, config.regularization, fieldName + ".regularization", verbose);

    if(config.regularization < 0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Regularization smaller than 0!");
    }

    loadData::loadPtreeValue(pt, config.hessianDiagonalShift, fieldName + ".hessianDiagonalShift", verbose);
    
    if(config.hessianDiagonalShift < 0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Hessian diagonal shift smaller than 0!");
    }

    loadData::loadPtreeValue(pt, config.barrierSettings.mu, 
      fieldName + ".mu", verbose);

    if(config.barrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, config.barrierSettings.delta, 
      fieldName + ".delta", verbose);

    if(config.barrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[ForceFrictionConeSoftConstraint]: Relaxed barrier penalty delta smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }
      
    return config;
  }
} // namespace legged_locomotion_mpc