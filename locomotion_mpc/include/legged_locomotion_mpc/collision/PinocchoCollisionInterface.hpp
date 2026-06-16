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
 * Based on: Farbod Farshidian (https://github.com/leggedrobotics/ocs2)
 */

#ifndef __PINOCCHIO_COLLISION_INTERFACE_LEGGED_LOCOMOTION_MPC__
#define __PINOCCHIO_COLLISION_INTERFACE_LEGGED_LOCOMOTION_MPC__

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sphere_approximation/SphereApproximation.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/collision/CollisionSettings.hpp>

namespace legged_locomotion_mpc
{
  namespace collision
  {
    /**
    * Interface for all collsion objects. 
    * 
    * There are two types of collisions, terrain collision and self collision. 
    * By default all end effectors are included to terrain collision. User can add new 
    * collision objects by link name in constructor.
    * 
    * Every object that user wants to add needs to have primitive collision, 
    * otherwise exception will be thrown.
    * 
    * Currently the following primitive geometries are supported: box, cylinder, and sphere.
    *
    * Reference:
    * [1] A. Voelz and K. Graichen, "Computation of Collision Distance and Gradient using an Automatic Sphere Approximation of the Robot Model
    * with Bounded Error," ISR 2018; 50th International Symposium on Robotics, 2018, pp. 1-8.
    * 
    * @warning 3 DoF end effector collison model i assumed to be sphere with center postion 
    * located in the same positon as end effector frame!
    */
    class PinocchioCollisionInterface
    {
      public:
        /**
         * Constructor
         * @param [in] info: Floating Base model info.
         * @param [in] modelSettings: model settings
         * @param [in] collisionSettings: collision settings
         * @param [in] pinocchioInterface: Pinocchio interface of Floating Base model
         * @param [in] verbose: print information.
         */
        PinocchioCollisionInterface(
          const floating_base_model::FloatingBaseModelInfo& info,
          const ModelSettings& modelSettings,
          const CollisionSettings& collisionSettings,
          const ocs2::PinocchioInterface& pinocchioInterface, bool verbose = true);
        
        /**
         * Get number of spheres in frame
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        size_t getFrameSphereNumber(size_t collisionIndex) const;

        /**
         * Get vector with radius of every sphere in every geometry in frame
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        const std::vector<ocs2::scalar_t>& getFrameSphereRadiuses(size_t collisionIndex) const;

        /**
         * Get vector with position of every sphere in every geometry with respect to frame
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        const std::vector<vector3_t>& getFrameSpherePositions(size_t collisionIndex) const;
        
        /**
         * Get vector with neighbours indices for queried sphere index
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        const std::vector<size_t>& getSphereNeighbours(size_t collisionIndex, 
          size_t sphereIndex) const;
        
        /**
         * Get all indices from range(0 : endEffectorNum + collisionNum - 1) 
         * of all terrain avoidance links
         */
        const std::vector<size_t>& getTerrainAvoidanceCollisionIndices() const;

        /**
         * Get all indices from range(0 : endEffectorNum + collisionNum - 1) 
         * of all self collision link pairs
         */
        const std::vector<std::pair<size_t, size_t>>& getSelfCollisionIndices() const;

        /**
         * Get Pinocchio geometry model
         */
        const pinocchio::GeometryModel& getGeometryModel() const;

      private:
        
        // Cpp AD version of rotation matrix and vector multiplication: R * v
        ocs2::ad_vector_t getRotationTimesVectorCppAd(
          const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesAD, 
          const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& vector);
        
        // Create geometryModel_
        void createGeometryModel(const ocs2::PinocchioInterface& pinocchioInterface);
        
        // Create terrainAvoidanceIndices_ and selfCollisionIndices_
        void createCollisionIndices(const floating_base_model::FloatingBaseModelInfo& info, 
          const ModelSettings& modelSettings, 
          const CollisionSettings& collisionSettings);
        
        // Create sphereNumbers_, sphereRadiuses_ and frameToSpherePositons_
        void createSphereDataStructs(const floating_base_model::FloatingBaseModelInfo& info, 
          const ModelSettings& modelSettings, 
          const CollisionSettings& collisionSettings, 
          const ocs2::PinocchioInterface& pinocchioInterface, bool verbose);
        
        // Create sphereNeighbours_
        void createNeighbours(const CollisionSettings& collisionSettings);

        pinocchio::GeometryModel geometryModel_;

        size_t frameNumber_;
        
        // Pinocchio indices of terrain collision avoidance
        std::vector<size_t> terrainAvoidanceIndices_;
        
        // Pinocchio indices of self collision avoidance
        std::vector<std::pair<size_t, size_t>> selfCollisionIndices_;
        
        // Number of spheres in every object of every frame
        std::vector<size_t> sphereNumbers_;

        // Radius values of spheres of every object of every frame
        std::vector<std::vector<ocs2::scalar_t>> sphereRadiuses_;

        // Positions of sphere centers relative to their parent frame
        std::vector<std::vector<vector3_t>> frameToSpherePositons_;

        /**
         * Data structure thats is used for getting closest neighbours of queried
         * collision sphere od queried collision link/end effector:
         * sphereNeighbours_[collisionLinkIndex][sphereIndex] -> neighbors (std::vector<sphereIndexes>)
         * Important: also includes queried sphere
         */
        std::vector<std::vector<std::vector<size_t>>> sphereNeighbours_;
    };
  } // namespace collision
} // namespace legged_locomotion_mpc

#endif