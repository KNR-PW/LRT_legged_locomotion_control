#include <legged_locomotion_mpc/soft_constraint/NormalOrientationSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <legged_locomotion_mpc/common/EulerTransforms.hpp>
#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;

  NormalOrientationSoftConstraint::NormalOrientationSoftConstraint(
    const LeggedReferenceManager& referenceManager,
    NormalOrientationSoftConstraint::Settings settings, FloatingBaseModelInfo info):
      referenceManager_(referenceManager),info_(std::move(info)),
      normalRelaxedBarrierPenaltyPtr_(new QuadraticPenalty(settings.barrierMu)) {}

  NormalOrientationSoftConstraint* NormalOrientationSoftConstraint::clone() const
  {
    return new NormalOrientationSoftConstraint(*this);
  }

  scalar_t NormalOrientationSoftConstraint::getValue(scalar_t time, const vector_t& state,
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    scalar_t cost = 0.0;

    for(size_t i = 0; i < info_.numSixDofContacts; ++i)
    {
      const size_t endEffectorIndex = info_.numThreeDofContacts + i;

      if(!contactFlags[endEffectorIndex]) continue;

      const auto& eulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        endEffectorIndex);

      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerAngles);

      const vector3_t& surfaceNormal = leggedPrecomputation.getSurfaceNormal(
        endEffectorIndex);

      const vector3_t endEffectorNormal = rotationMatrix.col(2);

      const scalar_t value = endEffectorNormal.dot(surfaceNormal) - 1.0;

      cost += normalRelaxedBarrierPenaltyPtr_->getValue(0.0, value);
    }
    
    return cost;
  }
        
  ScalarFunctionQuadraticApproximation NormalOrientationSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    ScalarFunctionQuadraticApproximation cost;

    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);

    for(size_t i = 0; i < info_.numSixDofContacts; ++i)
    {
      const size_t endEffectorIndex = info_.numThreeDofContacts + i;

      if(!contactFlags[endEffectorIndex]) continue;

      const auto& eulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        endEffectorIndex);

      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerAngles);

      const vector3_t& surfaceNormal = leggedPrecomputation.getSurfaceNormal(
        endEffectorIndex);

      const vector3_t endEffectorNormal = rotationMatrix.col(2);

      const scalar_t value = endEffectorNormal.dot(surfaceNormal) - 1.0;

      cost.f += normalRelaxedBarrierPenaltyPtr_->getValue(0.0, value);

      const scalar_t penaltyDerivative = normalRelaxedBarrierPenaltyPtr_->getDerivative(
        0.0, value);

      // Normal is third column of rotation matrix: dn_i / de = dR_i2 / de
      const auto rotationGradient = euler_transforms::getRotationMatrixEulerZyxGradient(
        eulerAngles);
      matrix3_t normalGradient;
      normalGradient.row(0) = rotationGradient[0][2].transpose();
      normalGradient.row(1) = rotationGradient[1][2].transpose();
      normalGradient.row(2) = rotationGradient[2][2].transpose();

      const matrix_t eulerAnglesDerivative = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        endEffectorIndex).dfdx.block(0, 6, 3, info_.generalizedCoordinatesNum);

      const vector_t dValueDx = eulerAnglesDerivative.transpose() * normalGradient.transpose() * surfaceNormal;

      cost.dfdx.block(6, 0, info_.generalizedCoordinatesNum, 1).noalias() 
        += penaltyDerivative * dValueDx;

      const scalar_t secondPenaltyDerivative = 
        normalRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, value);

      cost.dfdxx.block(6, 6, info_.generalizedCoordinatesNum, 
        info_.generalizedCoordinatesNum).noalias() 
          += secondPenaltyDerivative * dValueDx * dValueDx.transpose();
    }

    return cost;
  }

  NormalOrientationSoftConstraint::NormalOrientationSoftConstraint(
    const NormalOrientationSoftConstraint &other): referenceManager_(other.referenceManager_), 
      info_(other.info_), 
      normalRelaxedBarrierPenaltyPtr_(other.normalRelaxedBarrierPenaltyPtr_->clone()) {}

  NormalOrientationSoftConstraint::Settings loadNormalOrientationSoftConstraintSettings(
    const std::string& filename, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    NormalOrientationSoftConstraint::Settings settings;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Normal Orientation Soft Constraintt Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, settings.barrierMu, 
        fieldName + ".mu", verbose);

    if(settings.barrierMu < 0.0)
    {
      throw std::invalid_argument("[NormalOrientationSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc