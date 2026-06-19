#include <legged_locomotion_mpc/soft_constraint/JointLimitsSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <floating_base_model/AccessHelperFunctions.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsSoftConstraint::JointLimitsSoftConstraint(
    floating_base_model::FloatingBaseModelInfo info, 
    vector_t jointNominalPositions, 
    vector_t jointPositionUpperLimits, 
    vector_t jointPositionLowerLimits, 
    vector_t jointNominalVelocities, 
    vector_t jointVelocityLimits, 
    JointLimitsSoftConstraint::Settings settings):
      StateInputCost(),
      jointPositionLowerLimits_(std::move(jointPositionLowerLimits)), 
      jointPositionUpperLimits_(std::move(jointPositionUpperLimits)),
      jointVelocityLimits_(std::move(jointVelocityLimits)),
      info_(std::move(info)),
      jointRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings.barrierSettings)) 
  {
    if(jointNominalPositions.size() != info_.actuatedDofNum)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Wrong size of nominal poisitions!");
    }
    if(jointPositionLowerLimits_.size() != info_.actuatedDofNum)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Wrong size of position lower limits!");
    }
    if(jointPositionUpperLimits_.size() != info_.actuatedDofNum)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Wrong size of positon upper limits!");
    }
    if(jointNominalVelocities.size() != info_.actuatedDofNum)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Wrong size of nominal velocities!");
    }
    if(jointVelocityLimits_.size() != info_.actuatedDofNum)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Wrong size of velocity limits!");
    }

    /**
     * Get offset
     * Taking a penalty on constraint bounds that are far away can create a large 
     * (negative) value inside the feasible set, for example when
     * using a relaxed barrier constraint. Adding a constant 
     * offset does not change the optimal solution, but gives peace of mind that the
     * cost values are in a reasonable absolute range.
     */

    const vector_t upperBoundJointPositionOffset = jointPositionUpperLimits_ - jointNominalPositions;
    const vector_t lowerBoundJointPositionOffset = jointNominalPositions - jointPositionLowerLimits_;
    const vector_t upperBoundJointVelocitiesOffset = jointVelocityLimits_ - jointNominalVelocities;
    const vector_t lowerBoundJointVelocitiesOffset = jointVelocityLimits_ + jointNominalVelocities;

    jointPositionVelocityOffset_ = -1.0 * (upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsSoftConstraint* JointLimitsSoftConstraint::clone() const
  {
    return new JointLimitsSoftConstraint(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t JointLimitsSoftConstraint::getValue(scalar_t time, const vector_t& state,
    const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto jointPositions = access_helper_functions::getJointPositions(state, info_);
    const auto jointVelocities = access_helper_functions::getJointVelocities(input, info_);
    const vector_t upperBoundJointPositionOffset = jointPositionUpperLimits_ - jointPositions;
    const vector_t lowerBoundJointPositionOffset = jointPositions - jointPositionLowerLimits_;
    const vector_t upperBoundJointVelocitiesOffset = jointVelocityLimits_ - jointVelocities;
    const vector_t lowerBoundJointVelocitiesOffset = jointVelocityLimits_ + jointVelocities;

    const scalar_t cost = upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + jointPositionVelocityOffset_;

    return cost;
  }

  ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const
  {
    const auto jointPositions = access_helper_functions::getJointPositions(state, info_);
    const auto jointVelocities = access_helper_functions::getJointVelocities(input, info_);
    const vector_t upperBoundJointPositionOffset = jointPositionUpperLimits_ - jointPositions;
    const vector_t lowerBoundJointPositionOffset = jointPositions - jointPositionLowerLimits_;
    const vector_t upperBoundJointVelocitiesOffset = jointVelocityLimits_ - jointVelocities;
    const vector_t lowerBoundJointVelocitiesOffset = jointVelocityLimits_ + jointVelocities;

    ScalarFunctionQuadraticApproximation cost;

    cost.f = upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + jointPositionVelocityOffset_;

    const vector_t penaltyDerivativesState = 
      -upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      }) + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      });
    
    const vector_t penaltyDerivativesInput = 
      - upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      }) + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      });
    
    // Should be diagonal matrix, but will be stored as vector for faster computation
    const vector_t penaltySecondDerivativesState = 
      upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      }) + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });
    
     const vector_t penaltySecondDerivativesInput = 
      upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      }) + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });

    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdu = vector_t::Zero(info_.inputDim);

    const size_t forceSize = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;

    cost.dfdx.block(12, 0, info_.actuatedDofNum, 1) = penaltyDerivativesState;
    cost.dfdu.block(forceSize, 0, info_.actuatedDofNum, 1) = penaltyDerivativesInput;

    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
    cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);
    cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);

    cost.dfdxx.block(12, 12, info_.actuatedDofNum, info_.actuatedDofNum).diagonal() = 
      penaltySecondDerivativesState;

    cost.dfduu.block(forceSize, forceSize, info_.actuatedDofNum, 
      info_.actuatedDofNum).diagonal() = penaltySecondDerivativesInput;

    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint &rhs):
    StateInputCost(), info_(rhs.info_),
    jointPositionLowerLimits_(rhs.jointPositionLowerLimits_),
    jointPositionUpperLimits_(rhs.jointPositionUpperLimits_),
    jointVelocityLimits_(rhs.jointVelocityLimits_),
    jointRelaxedBarrierPenaltyPtr_(rhs.jointRelaxedBarrierPenaltyPtr_->clone()),
    jointPositionVelocityOffset_(rhs.jointPositionVelocityOffset_) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  using Settings = JointLimitsSoftConstraint::Settings;
  Settings loadJointLimitsSoftConstraintSettings(const std::string& filename,
    const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    Settings settings;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Joint Limits Soft Constraint Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.mu, 
        fieldName + ".mu", verbose);

    if(settings.barrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.delta, 
        fieldName + ".delta", verbose);

    if(settings.barrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[JointLimitsSoftConstraint]: Relaxed barrier penalty delta smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }

} // namespace legged_locomotion_mpc