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

#include <random>

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
      endEffectorAvoidancePenaltyPtr_(new RelaxedBarrierPenalty(
        settings.endEffectorBarrierSettings)),
      collisionLinksAvoidancePenaltyPtr_(new RelaxedBarrierPenalty(
        settings.collisionLinksBarrierSettings)) 
    {
      // Get random index of sphere for every 6 DoF end effector and collision link
      const size_t multipleSpheresSize = info.numSixDofContacts 
        + collisionLinkindices_.size();

      std::random_device randomDevice;
      std::mt19937 generator(randomDevice());

      currentCollisionLinksSphere_.reserve(multipleSpheresSize);

      // Six Dof End Effectors (many spheres)
      for(size_t i = info.numThreeDofContacts; i < endEffectorNum_; ++i)
      {
        const size_t numberOfSpheres = collisionInterface_.getFrameSphereNumber(i);
        std::uniform_int_distribution<> distribution(0, numberOfSpheres - 1);
        currentCollisionLinksSphere_.push_back(distribution(generator));
      }

      // Collison links (one or many spheres)
      for(size_t i = 0; i < collisionLinkindices_.size(); ++i)
      {
        const size_t collisionIndex = collisionLinkindices_[i];
        const size_t numberOfSpheres = collisionInterface_.getFrameSphereNumber(collisionIndex );
        std::uniform_int_distribution<> distribution(0, numberOfSpheres - 1);
        currentCollisionLinksSphere_.push_back(distribution(generator));
      }
    }

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
      cost += endEffectorAvoidancePenaltyPtr_->getValue(0.0, distance);
    }

    // Six Dof End Effectors (many spheres)
    for(size_t i = threeDofEndEffectorNum_; i < endEffectorNum_; ++i)
    {
      // If end effector is in contact, does not add cost
      if(contactFlags[i]) continue;
      const auto& radiuses = collisionInterface_.getFrameSphereRadiuses(i);
      const auto& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(i);
      const auto& framePosition = leggedPrecomputation.getEndEffectorPosition(i);
      const auto& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      size_t currentBestSphereIndex = currentCollisionLinksSphere_[i 
        - threeDofEndEffectorNum_];
      const auto& neighbourIndexes = collisionInterface_.getSphereNeighbours(i, 
        currentBestSphereIndex);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      for(const auto neighbourIndex: neighbourIndexes)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[neighbourIndex];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[neighbourIndex];
        if(distance < minDistance)
        {
          minDistance = distance;
          currentBestSphereIndex = neighbourIndex;
        }
      }
      currentCollisionLinksSphere_[i - threeDofEndEffectorNum_] = currentBestSphereIndex;
      
      const scalar_t relaxation = relaxations_[i];
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      
      cost += endEffectorAvoidancePenaltyPtr_->getValue(0.0, 
        minDistance - terrainClearance + relaxation);
    }

    // Collison links (one or many spheres)
    for(size_t i = 0; i < collisionLinkindices_.size(); ++i)
    {
      const size_t collisionIndex = collisionLinkindices_[i];
      const auto& radiuses = collisionInterface_.getFrameSphereRadiuses(
        collisionIndex);
      const auto& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(collisionIndex);
      const auto& framePosition = leggedPrecomputation.getCollisionLinkPosition(i);
      const auto& frameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      size_t currentBestSphereIndex = currentCollisionLinksSphere_[i + sixDofEndEffectorNum_];
      const auto& neighbourIndexes = collisionInterface_.getSphereNeighbours(collisionIndex, 
        currentBestSphereIndex);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      for(const auto neighbourIndex: neighbourIndexes)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[neighbourIndex];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[neighbourIndex];
        if(distance < minDistance)
        {
          minDistance = distance;
          currentBestSphereIndex = neighbourIndex;
        }
      }
      currentCollisionLinksSphere_[i + sixDofEndEffectorNum_] = currentBestSphereIndex;

      const scalar_t relaxation = relaxations_[i + endEffectorNum_];
      cost += collisionLinksAvoidancePenaltyPtr_->getValue(0.0, 
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

      cost.f += endEffectorAvoidancePenaltyPtr_->getValue(0.0, distance);

      const scalar_t penaltyDerivative = endEffectorAvoidancePenaltyPtr_->getDerivative(
        0.0, distance);

      const scalar_t penaltySecondDerivative = endEffectorAvoidancePenaltyPtr_->getSecondDerivative(
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
      const auto& radiuses = collisionInterface_.getFrameSphereRadiuses(i);
      const auto& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(i);
      const auto& framePosition = leggedPrecomputation.getEndEffectorPosition(i);
      const auto& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      size_t currentBestSphereIndex = currentCollisionLinksSphere_[i 
        - threeDofEndEffectorNum_];
      const auto& neighbourIndexes = collisionInterface_.getSphereNeighbours(i, 
        currentBestSphereIndex);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      for(const auto neighbourIndex: neighbourIndexes)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[neighbourIndex];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[neighbourIndex];
        if(distance < minDistance)
        {
          minDistance = distance;
          currentBestSphereIndex = neighbourIndex;
        }
      }
      currentCollisionLinksSphere_[i - threeDofEndEffectorNum_] = currentBestSphereIndex;
      
      const scalar_t relaxation = relaxations_[i];
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      minDistance += - terrainClearance + relaxation;
      
      cost.f += endEffectorAvoidancePenaltyPtr_->getValue(0.0, 
        minDistance);

      const vector3_t minSpherePosition = framePosition + rotationMatrix * sphereRelativePositions[currentBestSphereIndex];

      const vector3_t sdfGradient = sdf->derivative(minSpherePosition);

      const scalar_t penaltyDerivative = endEffectorAvoidancePenaltyPtr_->getDerivative(
        0.0, minDistance);

      const scalar_t penaltySecondDerivative = endEffectorAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, minDistance);

      const auto& positionDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);
      const auto& eulerDerivative = leggedPrecomputation.getEndEffectorOrientationDerivatives(i);

      const auto rotationVectorGradient = 
        leggedPrecomputation.getRotationTimesVectorGradient(frameEulerAngles, 
          sphereRelativePositions[currentBestSphereIndex]);

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
      const auto& radiuses = collisionInterface_.getFrameSphereRadiuses(
        collisionIndex);
      const auto& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(
        collisionIndex);
      const auto& framePosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);
      const auto& frameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        collisionIndex);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      size_t currentBestSphereIndex = currentCollisionLinksSphere_[i + sixDofEndEffectorNum_];
      const auto& neighbourIndexes = collisionInterface_.getSphereNeighbours(collisionIndex, 
        currentBestSphereIndex);
      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      for(const auto neighbourIndex: neighbourIndexes)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[neighbourIndex];
        const scalar_t distance = sdf->value(spherePositionInWorld) - radiuses[neighbourIndex];
        if(distance < minDistance)
        {
          minDistance = distance;
          currentBestSphereIndex = neighbourIndex;
        }
      }
      currentCollisionLinksSphere_[i + sixDofEndEffectorNum_] = currentBestSphereIndex;
      
      const scalar_t relaxation = relaxations_[i + endEffectorNum_];

      minDistance += relaxation;
      
      cost.f += collisionLinksAvoidancePenaltyPtr_->getValue(0.0, minDistance);

      const vector3_t minSpherePosition = framePosition + rotationMatrix * sphereRelativePositions[currentBestSphereIndex];

      const vector3_t sdfGradient = sdf->derivative(minSpherePosition);

      const scalar_t penaltyDerivative = collisionLinksAvoidancePenaltyPtr_->getDerivative(
        0.0, minDistance);

      const scalar_t penaltySecondDerivative = collisionLinksAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, minDistance);

      const auto& positionDerivative = leggedPrecomputation.getCollisionLinkPositionDerivatives(collisionIndex);
      const auto& eulerDerivative = leggedPrecomputation.getCollisionLinkOrientationDerivatives(collisionIndex);

      const auto rotationVectorGradient = 
        leggedPrecomputation.getRotationTimesVectorGradient(frameEulerAngles, 
          sphereRelativePositions[currentBestSphereIndex]);

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
      currentCollisionLinksSphere_(rhs.currentCollisionLinksSphere_),
      referenceManager_(rhs.referenceManager_),
      collisionInterface_(rhs.collisionInterface_),
      relaxations_(rhs.relaxations_),
      endEffectorAvoidancePenaltyPtr_(rhs.endEffectorAvoidancePenaltyPtr_->clone()),
      collisionLinksAvoidancePenaltyPtr_(rhs.collisionLinksAvoidancePenaltyPtr_->clone()) {}

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

    loadData::loadPtreeValue(pt, settings.endEffectorBarrierSettings.mu, 
        fieldName + ".endEffectorSettings.mu", verbose);

    if(settings.endEffectorBarrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[TerrainAvoidanceSoftConstraint]: Relaxed barrier penalty mu for end effectors smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.endEffectorBarrierSettings.delta, 
        fieldName + ".endEffectorSettings.delta", verbose);

    if(settings.endEffectorBarrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[TerrainAvoidanceSoftConstraint]: Relaxed barrier penalty delta for end effectors smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.collisionLinksBarrierSettings.mu, 
        fieldName + ".collisionLinksSettings.mu", verbose);

    if(settings.collisionLinksBarrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[TerrainAvoidanceSoftConstraint]: Relaxed barrier penalty mu for collision links smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.collisionLinksBarrierSettings.delta, 
        fieldName + ".collisionLinksSettings.delta", verbose);

    if(settings.collisionLinksBarrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[TerrainAvoidanceSoftConstraint]: Relaxed barrier penalty delta for collision links smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc
