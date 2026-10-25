//
// Created by rgrandia on 21.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __TERRAIN_PLANE_TERRAIN_MODEL__
#define __TERRAIN_PLANE_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>

namespace terrain_model
{
  /**
   * Planar terrain represented by a single coordinate frame.
   * - The terrain frame is located at positionInWorld.
   * - The surface normal points in positive z direction in the terrain frame.
   */
  class TerrainPlane
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
      /** 
       * Default constructor
       */
      TerrainPlane();
  
      /** 
       * Itemwise constructor
       */
      TerrainPlane(vector3_t positionInWorld, matrix3_t orientationWorldToTerrain);
  
      /**
       * Getters and for position in world frame
       */
      const vector3_t& getPosition() const;

      /**
       * Getters and for orientation: world -> terrain 
       */
      const matrix3_t& getOrientationToTerrain() const;
  
      /** 
       * Returns the surface normal = z-axis of the terrain, 
       * the unit vector is represented in the world frame
       */
      vector3_t getSurfaceNormalInWorld() const;
  
      /** Converts a 3D position in world frame to a 3D position in the terrain frame. */
      vector3_t getPositionInTerrainFrameFromPositionInWorld(
        const vector3_t& positionWorld) const;
      
      /** Converts a 3D position in terrain frame to a 3D position in the world frame. */
      vector3_t getPositionInWorldFrameFromPositionInTerrain(
        const vector3_t& positionInTerrain) const;
      
      /** 
       * Returns the orthogonal signed distance between the 
       * terrain a 3D point represented in world frame. 
       */
      ocs2::scalar_t getTerrainSignedDistanceFromPositionInWorld(
        const vector3_t& positionWorld) const;
      
      /** Returns the orthogonal projection onto the terrain plane for a 3D position in world.
       *  The returned position is in world frame 
       */
      vector3_t projectPositionInWorldOntoPlane(const vector3_t& positionWorld) const;
      
      /**
       * Returns the projection along gravity onto the terrain
       *  plane for a 2D position in world. The returned position is in world frame.
       *  Plane can not be vertical (surface normal z != 0).
       */
      vector3_t projectPositionInWorldOntoPlaneAlongGravity(
        const vector2_t& positionXYWorld) const;
      
      /** 
       * Returns the projection along gravity onto the terrain 
       * plane for a 3D position in world. The returned position is in world frame 
       */
      vector3_t projectPositionInWorldOntoPlaneAlongGravity(const vector3_t& positionWorld) const;
      
      /**
       *  Returns the projection along gravity onto the terrain orientation for a vector represented in world frame
       *  This projection is such that the x-y components of the vector remain unchanged.
       *  The returned vector is still represented in the world frame.
       */
      vector3_t projectVectorInWorldOntoPlaneAlongGravity(const vector3_t& vectorInWorld) const;
      
      /**
       * Change heading vector (X axis) of plane rotation matrix. It is first mapped onto plane,
       * then Y axis is calculated with right hand rule. Vertical or zero heading vector
       * does not change the orientation.
       */
      void changeHeadingVector(const vector3_t headingVectorInWorld);
      /**
       * Constructs the x-y unit vectors for a given z-axis. The absolute orientation of the x-y vectors is unspecified.
       * @param surfaceNormal (z-axis) of the terrain.
       * @return 2x3 matrix forming the [x-axis; y-axis] of the terrain
       */
      static Eigen::Matrix<ocs2::scalar_t, 2, 3> getTangentialBasisFromSurfaceNormal(
        const vector3_t& surfaceNormal);
      
      /**
       * Constructs a rotation matrix from the specified surface normal (z-axis). The rotation around the surface normal is unspecified.
       * @param surfaceNormal (z-axis) of the terrain.
       * @return Rotation matrix world to terrain, where the terrain frame has the z-axis aligned with the specified surface normal.
       */
      static matrix3_t getOrientationWorldToTerrainFromSurfaceNormalInWorld(
        const vector3_t& surfaceNormal);
      
    private:  

      vector3_t positionInWorld_;
      matrix3_t orientationWorldToTerrain_;
  };
}; // namespace terrain_model

#endif
