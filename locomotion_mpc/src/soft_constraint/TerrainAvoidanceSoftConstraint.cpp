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

#include <legged_locomotion_mpc/soft_constraint/TerrainAvoidanceSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace collision;
  using namespace terrain_model;

  TerrainAvoidanceSoftConstraint::TerrainAvoidanceSoftConstraint(
    FloatingBaseModelInfo info,
    const CollisionSettings& collisionSettings,
    const PinocchioCollisionInterface& collisionInterface,
    const LeggedReferenceManager& referenceManager,
    Settings settings):
      threeDofEndEffectorNum_(info.numThreeDofContacts),
      sixDofEndEffectorNum_(info.numSixDofContacts),
      endEffectorNum_(info.numThreeDofContacts + info.numSixDofContacts),
      collisionLinkindices_(collisionInterface.getTerrainAvoidanceCollisionIndices()),
      relaxations_(collisionSettings.terrainRelaxations),
      referenceManager_(referenceManager),
      collisionInterface_(collisionInterface),
      terrainAvoidancePenaltyPtr_(new RelaxedBarrierPenalty(settings.barrierSettings)) {}

  TerrainAvoidanceSoftConstraint* TerrainAvoidanceSoftConstraint::clone() const
  {
    return new TerrainAvoidanceSoftConstraint(*this);
  }

  scalar_t TerrainAvoidanceSoftConstraint::getValue(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, 
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const SignedDistanceField* sdf = referenceManager_.getTerrainModel()
      .getSignedDistanceField();
    
    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    scalar_t cost = 0.0;

    // Three Dof End Effectors (one sphere)
    for(size_t i = 0; i < threeDofEndEffectorNum_; ++i)
    {
      // If end effector is in contact, does not add cost
      if(contactFlags[i]) continue;
      const scalar_t radius = collisionInterface_.getFrameSphereRadiuses(i)[0];
      const vector3_t& position = leggedPrecomputation.getEndEffectorPosition(i);
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      const scalar_t relaxation = relaxations_[i];
      const scalar_t distance = sdf->value(position) - terrainClearance
        - radius + relaxation;
      cost += terrainAvoidancePenaltyPtr_->getValue(0.0, distance);
    }

    // Six Dof End Effectors (many spheres)
    for(size_t i = threeDofEndEffectorNum_; i < endEffectorNum_; ++i)
    {
      // If end effector is in contact, does not add cost
      if(contactFlags[i]) continue;
      const std::vector<scalar_t>& radiuses = collisionInterface_.getFrameSphereRadiuses(i);
      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(i);
      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(i);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      for(size_t j = 0; j < sphereRelativePositions.size(); ++j)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[j];
        if(distance < minDistance) minDistance = distance;
      }      
      const scalar_t relaxation = relaxations_[i];
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      
      cost += terrainAvoidancePenaltyPtr_->getValue(0.0, 
        minDistance - terrainClearance + relaxation);
    }

    // Collison links (one or many spheres)
    for(size_t i = 0; i < collisionLinkindices_.size(); ++i)
    {
      const size_t collisionIndex = collisionLinkindices_[i];
      const std::vector<scalar_t>& radiuses = collisionInterface_.getFrameSphereRadiuses(
        collisionIndex);
      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(collisionIndex);
      const vector3_t& framePosition = leggedPrecomputation.getCollisionLinkPosition(i);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      for(size_t j = 0; j < sphereRelativePositions.size(); ++j)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[j];
        if(distance < minDistance) minDistance = distance;
      }      
      const scalar_t relaxation = relaxations_[i + endEffectorNum_];
      cost += terrainAvoidancePenaltyPtr_->getValue(0.0, 
        minDistance + relaxation);
    }
    return cost;
  }

      
  ScalarFunctionQuadraticApproximation TerrainAvoidanceSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const SignedDistanceField* sdf = referenceManager_.getTerrainModel()
      .getSignedDistanceField();

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(state.size());
    cost.dfdxx = matrix_t::Zero(state.size(), state.size());

    // Three Dof End Effectors (one sphere)
    for(size_t i = 0; i < threeDofEndEffectorNum_; ++i)
    {
      // If end effector is in contact, does not add cost
      if(contactFlags[i]) continue;
      const scalar_t radius = collisionInterface_.getFrameSphereRadiuses(i)[0];
      const vector3_t& position = leggedPrecomputation.getEndEffectorPosition(i);
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      const scalar_t relaxation = relaxations_[i];
      const auto [sdfDistance, sdfGradient] = sdf->valueAndDerivative(position);

      const scalar_t distance = sdfDistance - radius - terrainClearance + relaxation;

      cost.f += terrainAvoidancePenaltyPtr_->getValue(0.0, distance);

      const scalar_t penaltyDerivative = terrainAvoidancePenaltyPtr_->getDerivative(
        0.0, distance);

      const scalar_t penaltySecondDerivative = terrainAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, distance);

      const auto& positionDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);
      
      const vector_t scaledGradient = positionDerivative.dfdx.transpose() * sdfGradient;
      
      cost.dfdx.noalias() += penaltyDerivative * scaledGradient;

      // Approximated second derivative (sdf and postion second gradients are omitted)
      cost.dfdxx.noalias() += penaltySecondDerivative * scaledGradient * scaledGradient.transpose();
    }

    // Six Dof End Effectors (many spheres)
    for(size_t i = threeDofEndEffectorNum_; i < endEffectorNum_; ++i)
    {
      // If end effector is in contact, does not add cost
      if(contactFlags[i]) continue;
      const std::vector<scalar_t>& radiuses = collisionInterface_.getFrameSphereRadiuses(i);
      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(i);
      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(i);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minIndex = 0;
      for(size_t j = 0; j < sphereRelativePositions.size(); ++j)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[j];
        if(distance < minDistance)
        {
          minDistance = distance;
          minIndex = j;
        }
      }      

      const scalar_t relaxation = relaxations_[i];
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      minDistance += - terrainClearance + relaxation;
      
      cost.f += terrainAvoidancePenaltyPtr_->getValue(0.0, 
        minDistance);

      const vector3_t minSpherePosition = framePosition + rotationMatrix * sphereRelativePositions[minIndex];

      const vector3_t sdfGradient = sdf->derivative(minSpherePosition);

      const scalar_t penaltyDerivative = terrainAvoidancePenaltyPtr_->getDerivative(
        0.0, minDistance);

      const scalar_t penaltySecondDerivative = terrainAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, minDistance);

      const auto& positionDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);
      const auto& eulerDerivative = leggedPrecomputation.getEndEffectorOrientationDerivatives(i);

      const auto rotationVectorGradient = 
        leggedPrecomputation.getRotationTimesVectorGradient(frameEulerAngles, 
          sphereRelativePositions[minIndex]);

      const matrix_t positionStateDerivative = positionDerivative.dfdx + rotationVectorGradient * eulerDerivative.dfdx;
      const vector_t scaledGradient = positionStateDerivative.transpose() * sdfGradient;
      
      cost.dfdx.noalias() += penaltyDerivative * scaledGradient;

      // Approximated second derivative (sdf and postion second gradients are omitted)
      cost.dfdxx.noalias() += penaltySecondDerivative * scaledGradient * scaledGradient.transpose();
    }

    // Collison links (one or many spheres)
    for(size_t i = 0; i < collisionLinkindices_.size(); ++i)
    {
      const size_t collisionIndex = collisionLinkindices_[i];
      const std::vector<scalar_t>& radiuses = collisionInterface_.getFrameSphereRadiuses(
        collisionIndex);
      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(collisionIndex);
      const vector3_t& framePosition = leggedPrecomputation.getCollisionLinkPosition(collisionIndex);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(collisionIndex);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minIndex = 0;

      for(size_t j = 0; j < sphereRelativePositions.size(); ++j)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[j];
        if(distance < minDistance)
        {
          minDistance = distance;
          minIndex = j;
        }
      }      
      const scalar_t relaxation = relaxations_[i + endEffectorNum_];

      minDistance += relaxation;
      
      cost.f += terrainAvoidancePenaltyPtr_->getValue(0.0, minDistance);

      const vector3_t minSpherePosition = framePosition + rotationMatrix * sphereRelativePositions[minIndex];

      const vector3_t sdfGradient = sdf->derivative(minSpherePosition);

      const scalar_t penaltyDerivative = terrainAvoidancePenaltyPtr_->getDerivative(
        0.0, minDistance);

      const scalar_t penaltySecondDerivative = terrainAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, minDistance);

      const auto& positionDerivative = leggedPrecomputation.getCollisionLinkPositionDerivatives(collisionIndex);
      const auto& eulerDerivative = leggedPrecomputation.getCollisionLinkOrientationDerivatives(collisionIndex);

      // TOOOOOOOOOOOOOOOOOOOOOOOOO
      const auto rotationVectorGradient = 
        leggedPrecomputation.getRotationTimesVectorGradient(frameEulerAngles, 
          sphereRelativePositions[minIndex]);

      const matrix_t positionStateDerivative = positionDerivative.dfdx + rotationVectorGradient * eulerDerivative.dfdx;
      const vector_t scaledGradient = positionStateDerivative.transpose() * sdfGradient;
      
      cost.dfdx.noalias() += penaltyDerivative * scaledGradient;

      // Approximated second derivative (sdf and postion second gradients are omitted)
      cost.dfdxx.noalias() += penaltySecondDerivative * scaledGradient * scaledGradient.transpose();
    }
    return cost;
  }

  TerrainAvoidanceSoftConstraint::TerrainAvoidanceSoftConstraint(
    const TerrainAvoidanceSoftConstraint& rhs):
      threeDofEndEffectorNum_(rhs.threeDofEndEffectorNum_),
      sixDofEndEffectorNum_(rhs.sixDofEndEffectorNum_),
      endEffectorNum_(rhs.endEffectorNum_),
      collisionLinkindices_(rhs.collisionLinkindices_),
      referenceManager_(rhs.referenceManager_),
      collisionInterface_(rhs.collisionInterface_),
      relaxations_(rhs.relaxations_),
      terrainAvoidancePenaltyPtr_(rhs.terrainAvoidancePenaltyPtr_->clone()) {}

  using Settings = TerrainAvoidanceSoftConstraint::Settings;
  Settings loadTerrainAvoidanceSoftConstraintSettings(
    const std::string& filename, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    Settings settings;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Terrain Avoidance Soft Constraint Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.mu, 
        fieldName + ".mu", verbose);

    if(settings.barrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[TerrainAvoidanceSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.delta, 
        fieldName + ".delta", verbose);

    if(settings.barrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[TerrainAvoidanceSoftConstraint]: Relaxed barrier penalty delta smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc
