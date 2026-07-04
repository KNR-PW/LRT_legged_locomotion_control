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
   Based on: Farbod Farshidian (https://github.com/leggedrobotics/ocs2)
 */

#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>

#include <stdexcept>

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged_locomotion_mpc
{
  namespace collision
  {

    using namespace ocs2;
    using namespace floating_base_model;
    PinocchioCollisionInterface::PinocchioCollisionInterface(
      const FloatingBaseModelInfo& info,
      const ModelSettings& modelSettings,
      const CollisionSettings& collisionSettings,
      const PinocchioInterface& pinocchioInterface, bool verbose)
    {
      if(collisionSettings.maxExcesses.size() != (info.endEffectorFrameIndices.size() + collisionSettings.collisionLinkNames.size()))
      {
        throw std::invalid_argument("[PinocchioCollisionInterface]: "
          "maxExcesses is not the same size as collisionLinks!");
      }

      if(!pinocchioInterface.getUrdfModelPtr()) 
      {
        throw std::invalid_argument(
          "[PinocchioCollisionInterface]: The PinocchioInterface passed to "
          "PinocchioGeometryInterface(...) "
          "does not contain a urdf model!");
      } 

      createGeometryModel(pinocchioInterface);

      createSphereDataStructs(info, modelSettings, collisionSettings, pinocchioInterface, 
        verbose);

      createCollisionIndices(info, modelSettings, collisionSettings);

      createNeighbours(collisionSettings);
    }

    void PinocchioCollisionInterface::createGeometryModel(
      const PinocchioInterface& pinocchioInterface)
    {
      const std::unique_ptr<const TiXmlDocument> urdfAsXml(urdf::exportURDF(
        *pinocchioInterface.getUrdfModelPtr()));
      TiXmlPrinter printer;
      urdfAsXml->Accept(&printer);
      const std::stringstream urdfAsStringStream(printer.Str());

      geometryModel_ = pinocchio::GeometryModel();
      pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfAsStringStream, 
      pinocchio::COLLISION, geometryModel_);
    }

    void PinocchioCollisionInterface::createCollisionIndices(
      const FloatingBaseModelInfo& info,
      const ModelSettings& modelSettings, 
      const CollisionSettings& collisionSettings)
    {
      std::vector<std::string> collisionNames = modelSettings.endEffectorThreeDofNames;
      collisionNames.insert(collisionNames.end(), 
        modelSettings.endEffectorSixDofNames.begin(), modelSettings.endEffectorSixDofNames.end());

      collisionNames.insert(collisionNames.end(), 
        collisionSettings.collisionLinkNames.begin(), 
        collisionSettings.collisionLinkNames.end());

      terrainAvoidanceIndices_.clear();
      for(const auto& terrainLinkName: collisionSettings.terrainCollisionLinkNames)
      {
        const auto iter = std::find(collisionNames.cbegin(), collisionNames.cend(), terrainLinkName);
        if(iter == collisionNames.cend())
        {
          std::string message = "[PinocchioCollisionInterface]: No " + terrainLinkName + " link in all collison links!";
          throw std::invalid_argument(message);
        }
        const size_t index = std::distance(collisionNames.cbegin(), iter);
        terrainAvoidanceIndices_.push_back(index);
      }

      for(const auto& [firstCollisionName, secondCollisionName]: collisionSettings.selfCollisionPairNames)
      {
        const auto firstIter = std::find(collisionNames.cbegin(), collisionNames.cend(), firstCollisionName);
        const auto secondIter = std::find(collisionNames.cbegin(), collisionNames.cend(), secondCollisionName);

        if(firstIter == collisionNames.cend())
        {
          std::string message = "[PinocchioCollisionInterface]: No " + firstCollisionName + " link in all collison links!";
          throw std::invalid_argument(message);
        }

        if(secondIter == collisionNames.cend())
        {
          std::string message = "[PinocchioCollisionInterface]: No " + secondCollisionName + " link in all collison links!";
          throw std::invalid_argument(message);
        }

        const size_t firstIndex = std::distance(collisionNames.cbegin(), firstIter);
        const size_t secondIndex = std::distance(collisionNames.cbegin(), secondIter);

        if(secondIndex < firstIndex)
        {
          selfCollisionIndices_.emplace_back(secondIndex, firstIndex);
        }
        else
        {
          selfCollisionIndices_.emplace_back(firstIndex, secondIndex);
        }
      }
    }
        
    void PinocchioCollisionInterface::createSphereDataStructs(
      const FloatingBaseModelInfo& info, 
      const ModelSettings& modelSettings, 
      const CollisionSettings& collisionSettings, 
      const PinocchioInterface& pinocchioInterface, bool verbose)
    {
      // Get collision frame indexes
      std::vector<size_t> collisionFrames = info.endEffectorFrameIndices;

      std::vector<std::string> collisionNames = modelSettings.endEffectorThreeDofNames;
      collisionNames.insert(collisionNames.end(), 
        modelSettings.endEffectorSixDofNames.begin(), modelSettings.endEffectorSixDofNames.end());

      for(const std::string& frameName: collisionSettings.collisionLinkNames)
      {
        const size_t frameIndex = pinocchioInterface.getModel().getFrameId(frameName);
        if(frameIndex <= 0)
        {
          std::string message = "[PinocchioCollisionInterface]: There is no frame named " + frameName + "!";
          throw std::invalid_argument(message);
        }
        collisionFrames.push_back(frameIndex);
        collisionNames.push_back(frameName);
      }

      frameNumber_ = collisionFrames.size();

      for(size_t i = 0; i < frameNumber_; ++i)
      {
        const size_t frameIndex = collisionFrames[i];
        const auto& framePlacement = pinocchioInterface.getModel().frames[
          frameIndex].placement;

        size_t sphereNumber = 0;
        std::vector<scalar_t> sphereRadiuses;
        std::vector<vector3_t> spherePositions;

        for (size_t j = 0; j < geometryModel_.geometryObjects.size(); ++j) 
        {
          const pinocchio::GeometryObject& object = geometryModel_.geometryObjects[j];
          const size_t parentFrameIndex = object.parentFrame;

          if(parentFrameIndex == frameIndex) 
          {
            const auto& objectCenterPlacement = object.placement;
            const auto frameToCenterPlacement = framePlacement.actInv(objectCenterPlacement);
            const auto& frameToCenterPosition = frameToCenterPlacement.translation();
            const auto& centerToFrameRotation = frameToCenterPlacement.rotation();
            
            const SphereApproximation sphereApproximation = SphereApproximation(
              *object.geometry, j, collisionSettings.maxExcesses[i], 
              collisionSettings.shrinkRatio);

            const size_t objectSphereNumber = sphereApproximation.getNumSpheres();
            sphereNumber += objectSphereNumber;

            const scalar_t sphereRadius = sphereApproximation.getSphereRadius();
            sphereRadiuses.insert(sphereRadiuses.end(), objectSphereNumber, sphereRadius);

            const auto& centerToSpherePositions = sphereApproximation.getSphereCentersToObjectCenter();

            for(size_t k = 0; k < objectSphereNumber; ++k)
            {
              vector3_t frameToSphere = frameToCenterPosition + centerToFrameRotation * 
                centerToSpherePositions[k];
              spherePositions.push_back(std::move(frameToSphere));
            }
          }
        }
        sphereNumbers_.push_back(sphereNumber);
        sphereRadiuses_.push_back(std::move(sphereRadiuses));
        frameToSpherePositons_.push_back(std::move(spherePositions));
      }

      if(verbose)
      {
        for(size_t i = 0; i < collisionNames.size(); ++i)
        {
          std::string message = "[PinocchioCollisionInterface]: Collision frame named " + collisionNames[i] + " has "
            + std::to_string(sphereNumbers_[i]) + " spheres:\n";
          std::cerr << message;

          for(size_t j = 0; j < sphereNumbers_[i]; ++j)
          {
            std::cerr << "Sphere " << j << ": " << sphereRadiuses_[i][j] << ", [" << frameToSpherePositons_[i][j].transpose() << "]\n";
          }
        }
      }
    }
        
    void PinocchioCollisionInterface::createNeighbours(
      const CollisionSettings& collisionSettings)
    {
      sphereNeighbours_.reserve(frameNumber_);

      for(size_t i = 0; i < frameNumber_; ++i)
      {
        const size_t sphereNumber = sphereNumbers_[i];
        const auto& sphereRadiuses = sphereRadiuses_[i];
        const auto& frameToSpherePositons = frameToSpherePositons_[i];

        std::vector<std::vector<size_t>> frameSphereNeighbours;
        frameSphereNeighbours.reserve(sphereNumber);

        for(size_t j = 0; j < sphereNumber; ++j)
        {
          std::vector<size_t> sphereNeighbours;
          std::vector<std::pair<size_t, scalar_t>> distances;
          distances.reserve(sphereNumber);

          // Get all pairs of (index, distance) for this sphere
          for(size_t k = 0; k < sphereNumber; ++k)
          {
            const auto distancePair = std::make_pair(k, 
              (frameToSpherePositons[j] - frameToSpherePositons[k]).norm());
            distances.push_back(distancePair);
          }

          // Sort pairs of (index, distance)
          std::sort(distances.begin(), distances.end(), 
            [](const std::pair<size_t, scalar_t>& first, 
            const std::pair<size_t, scalar_t>& second)
            { return first.second < second.second;});
          
          // Get final neighbours
          const size_t numNeighbours = distances.size() > collisionSettings.maxSphereNeighbours ? collisionSettings.maxSphereNeighbours: distances.size();
          for(size_t k = 0; k < numNeighbours; ++k)
          {
            sphereNeighbours.push_back(distances[k].first);
          }
          frameSphereNeighbours.push_back(std::move(sphereNeighbours)); 
        }
        sphereNeighbours_.push_back(std::move(frameSphereNeighbours));
      }
    }

    size_t PinocchioCollisionInterface::getFrameSphereNumber(
      size_t collisionIndex) const
    {
      assert(collisionIndex < frameNumber_);
      return sphereNumbers_[collisionIndex];
    }

    const std::vector<scalar_t>& PinocchioCollisionInterface::getFrameSphereRadiuses(
      size_t collisionIndex) const
    {
      assert(collisionIndex < frameNumber_);
      return sphereRadiuses_[collisionIndex];
    }

    const std::vector<vector3_t>& PinocchioCollisionInterface::getFrameSpherePositions(
      size_t collisionIndex) const
    {
      assert(collisionIndex < frameNumber_);
      return frameToSpherePositons_[collisionIndex];
    }

    const std::vector<size_t>& PinocchioCollisionInterface::getSphereNeighbours(
      size_t collisionIndex, size_t sphereIndex) const
    {
      assert(collisionIndex < frameNumber_);
      assert(sphereIndex < sphereNumbers_[collisionIndex]);

      return sphereNeighbours_[collisionIndex][sphereIndex];
    }

    const pinocchio::GeometryModel& PinocchioCollisionInterface::getGeometryModel() const
    {
      return geometryModel_;
    }

    const std::vector<size_t>& PinocchioCollisionInterface::getTerrainAvoidanceCollisionIndices() const
    {
      return terrainAvoidanceIndices_;
    }

    const std::vector<std::pair<size_t, size_t>>& PinocchioCollisionInterface::getSelfCollisionIndices() const
    {
      return selfCollisionIndices_;
    }
  } // namespace collision
} // namespace legged_locomotion_mpc
