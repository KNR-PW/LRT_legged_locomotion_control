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
 * Based on: rgrandia (https://github.com/leggedrobotics/ocs2)
 */

#include <legged_locomotion_mpc/soft_constraint/JointTorqueLimitsSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointTorqueLimitsSoftConstraint::JointTorqueLimitsSoftConstraint(
    FloatingBaseModelInfo info, vector_t nominalTorque, 
    vector_t torqueLimits, JointTorqueLimitsSoftConstraint::Settings settings):
      StateInputCost(),
      info_(std::move(info)),
      torqueRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings.barrierSettings)),
      torqueLimits_(std::move(torqueLimits)), nominalTorqueOffset_(0.0)
  {
    /**
     * Get offset
     * Taking a penalty on constraint bounds that are far away can create a large 
     * (negative) value inside the feasible set, for example when
     * using a relaxed barrier constraint. Adding a constant 
     * offset does not change the optimal solution, but gives peace of mind that the
     * cost values are in a reasonable absolute range.
     */
    const vector_t upperBoundTorqueOffset = torqueLimits_ - nominalTorque;
    const vector_t lowerBoundTorqueOffset = torqueLimits_ + nominalTorque;

    nominalTorqueOffset_ = upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointTorqueLimitsSoftConstraint *JointTorqueLimitsSoftConstraint::clone() const
  {
    return new JointTorqueLimitsSoftConstraint(*this);
  }
    
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t JointTorqueLimitsSoftConstraint::getValue(scalar_t time, const vector_t& state,
    const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const vector_t& torqueApprox = leggedPrecomputation.getApproximatedJointTorques();
    
    const vector_t upperBoundTorqueOffset = torqueLimits_ - torqueApprox;
    const vector_t lowerBoundTorqueOffset = torqueLimits_ + torqueApprox;

    return upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + nominalTorqueOffset_;
  }
    
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ScalarFunctionQuadraticApproximation JointTorqueLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const VectorFunctionLinearApproximation& torqueApproxDerivative = 
      leggedPrecomputation.getApproximatedJointTorquesDerivatives();

    const vector_t upperBoundTorqueOffset = torqueLimits_ - torqueApproxDerivative.f;
    const vector_t lowerBoundTorqueOffset = torqueLimits_ + torqueApproxDerivative.f;

    const size_t forceSize = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
    
    const matrix_t dTorquedQ = torqueApproxDerivative.dfdx.block(0, 6, info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    const matrix_t dTorquedF = torqueApproxDerivative.dfdu.block(0, 0, info_.actuatedDofNum, forceSize);
    
    ScalarFunctionQuadraticApproximation cost;

    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdu = vector_t::Zero(info_.inputDim);
    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
    cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);
    cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);

    cost.f = upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
        return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + nominalTorqueOffset_;

    // Penalty derivatives w.r.t. the constraint
    const vector_t penaltyDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
        return torqueRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      }) - upperBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      { 
        return torqueRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi); 
      });
    
    // Should be diagonal matrix, but will be stored as vector for faster computation
    const vector_t penaltySecondDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
          return torqueRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      }) + upperBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
          return torqueRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });

    const matrix_t scaledDTorqueQ = penaltySecondDerivatives.asDiagonal() * dTorquedQ;

    cost.dfdx.block(6, 0, info_.generalizedCoordinatesNum, 1).noalias() =  dTorquedQ.transpose() * penaltyDerivatives;
    cost.dfdu.block(0, 0, forceSize, 1).noalias() =  dTorquedF.transpose() * penaltyDerivatives;
    cost.dfdxx.block(6, 6, info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum).noalias() = 
        dTorquedQ.transpose() * scaledDTorqueQ;

    cost.dfduu.block(0, 0, forceSize, forceSize).noalias() = 
      dTorquedF.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedF;
    
    cost.dfdux.block(0, 6, forceSize, info_.generalizedCoordinatesNum).noalias() = 
        dTorquedF.transpose() * scaledDTorqueQ;

    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointTorqueLimitsSoftConstraint::JointTorqueLimitsSoftConstraint(const JointTorqueLimitsSoftConstraint &rhs):
    StateInputCost(), info_(rhs.info_), 
    torqueRelaxedBarrierPenaltyPtr_(rhs.torqueRelaxedBarrierPenaltyPtr_->clone()),
    torqueLimits_(rhs.torqueLimits_), nominalTorqueOffset_(rhs.nominalTorqueOffset_) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  using Settings = JointTorqueLimitsSoftConstraint::Settings;
  Settings loadJointTorqueLimitsSoftConstraintSettings(
    const std::string& filename, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    Settings settings;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Joint Torque Limits Soft Constraint Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.mu, 
        fieldName + ".mu", verbose);

    if(settings.barrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[JointTorqueLimitsSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.delta, 
        fieldName + ".delta", verbose);

    if(settings.barrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[JointTorqueLimitsSoftConstraint]: Relaxed barrier penalty delta smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc