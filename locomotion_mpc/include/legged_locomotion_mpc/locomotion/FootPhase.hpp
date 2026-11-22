//
// Created by rgrandia on 24.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 01.09.2025 
//

#ifndef __FOOT_PHASE_LEGGED_LOCOMOTION_MPC__
#define __FOOT_PHASE_LEGGED_LOCOMOTION_MPC__

#include <memory>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/SwingSpline3d.hpp>

#include <terrain_model/core/ConvexTerrain.hpp>
#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/core/TerrainModel.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    /**
     * Linear inequality constraint A_p * p_world + b >=  0
     */
    struct FootTangentialConstraintMatrix 
    {
      Eigen::Matrix<ocs2::scalar_t, -1, 3> A;
      ocs2::vector_t b;
      
      bool isActive() const
      {
        return (A.rows() != 0 && b.rows() != 0);
      }
    };

    FootTangentialConstraintMatrix tangentialConstraintsFromConvexTerrain(
      const terrain_model::ConvexTerrain &stanceTerrain, ocs2::scalar_t margin);

    /**
     * Base class for a planned foot phase: Stance or Swing.
     */
    class FootPhase 
    {
      public:
        FootPhase() = default;

        virtual ~FootPhase() = default;

        FootPhase(const FootPhase &) = delete;

        FootPhase &operator=(const FootPhase &) = delete;

        /** Returns the contact flag for this phase. Stance phase: True, Swing phase: false */
        virtual bool contactFlag() const = 0;

        /** Returns the unit vector pointing in the normal direction.
         *  If stance mode, get terrain normal. When in motion, get interpolated normal
         * from previous and next terrain normals.
         */
        virtual vector3_t normalDirectionInWorldFrame(ocs2::scalar_t time) const = 0;

        /** Returns rotation matrix from world to terrain frame 
         * Call only when contactFlag() returns true!
         */
        virtual matrix3_t rotationMatrixInTerrainFrame(ocs2::scalar_t time) const = 0;

        /** Nominal foothold location (upcoming for swinglegs) */
        virtual vector3_t nominalFootholdLocation() const = 0;

        /** Convex terrain that constrains the foot placement. (null for swinglegs) */
        virtual const terrain_model::ConvexTerrain *nominalFootholdConstraint() const { return nullptr; };

        /** Foot reference position in world frame */
        virtual vector3_t getPositionInWorld(ocs2::scalar_t time) const = 0;

        /** Foot reference velocity in world frame */
        virtual vector3_t getVelocityInWorld(ocs2::scalar_t time) const = 0;

        /** Foot reference acceleration in world frame */
        virtual vector3_t getAccelerationInWorld(ocs2::scalar_t time) const = 0;

        /** Returns the position inequality constraints formulated in the tangential direction */
        virtual const FootTangentialConstraintMatrix *getFootTangentialConstraintInWorldFrame() const 
        {
          return nullptr;
        };

        virtual ocs2::scalar_t getMinimumFootClearance(ocs2::scalar_t time) const { return 0.0; };
    };

    /**
     * Encodes a planned stance phase on a terrain plane.
     * The normal constraint makes the foot converge to the terrain plane when positionGain > 0.0
     */
    class StancePhase final: public FootPhase 
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit StancePhase(terrain_model::ConvexTerrain stanceTerrain, ocs2::scalar_t terrainMargin = 0.0);

        ~StancePhase() override = default;

        bool contactFlag() const override { return true; };

        vector3_t normalDirectionInWorldFrame(ocs2::scalar_t time) const override;

        matrix3_t rotationMatrixInTerrainFrame(ocs2::scalar_t time) const override;

        vector3_t nominalFootholdLocation() const override;

        const terrain_model::ConvexTerrain *nominalFootholdConstraint() const override;

        vector3_t getPositionInWorld(ocs2::scalar_t time) const override;

        vector3_t getVelocityInWorld(ocs2::scalar_t time) const override;

        vector3_t getAccelerationInWorld(ocs2::scalar_t time) const override;

        const FootTangentialConstraintMatrix *getFootTangentialConstraintInWorldFrame() const override;

      private:
        terrain_model::ConvexTerrain stanceTerrain_;
        const matrix3_t rotationMatrixToTerrain_;
        const vector3_t nominalFootholdLocation_;
        const vector3_t surfaceNormalInWorldFrame_;
        const FootTangentialConstraintMatrix footTangentialConstraint_;
    };

    /**
     * Encodes a swing trajectory between two terrain planes.
     * A spline based swing motion is designed in both liftoff and target plane.
     */
    class SwingPhase final: public FootPhase 
    {
      public:
        struct SwingEvent 
        {
          ocs2::scalar_t time;
          ocs2::scalar_t velocity;
          const terrain_model::TerrainPlane *terrainPlane;
        };

        struct SwingProfile 
        {
          struct Node 
          {
           /** Time progress in the swing phase in [0, 1] */
            ocs2::scalar_t phase = 0.5;
           /** Swing height in normal direction */
            ocs2::scalar_t swingHeight = 0.15;
           /** Velocity in normal direction */
            ocs2::scalar_t normalVelocity = 0.0;
           /** Swing progress in tangential direction in [0, 1] */
            ocs2::scalar_t tangentialProgress = 0.5;
           /** Tangantial velocity as a factor of the average velocity.
            *  The tangential velocity will be: velocityFactor * swingdistance / dt 
            */
            ocs2::scalar_t tangentialVelocityFactor = 1.0;
          };

         /** Height / velocity profile */
          std::vector<Node> nodes;
         /** Desired SDF clearance at the middle of the swing phase. */
          ocs2::scalar_t sdfMidswingMargin = 0.0;
         /** Desired SDF clearance at liftoff and touchdown.
          *  Slight negative margin allows a bit of ground penetration.
          */
          ocs2::scalar_t sdfStartEndMargin = -0.02;
         /** Limits the amount of additional swing height from terrain adaptation. */
          ocs2::scalar_t maxSwingHeightAdaptation = 0.2;
         /**
          * Height of the end effector frame above the terrain at liftoff and touchdown [m].
          * Obstacles are measured from the terrain below the swing line.
          */
          ocs2::scalar_t endEffectorHeightAboveTerrain = 0.0;
        };

        /**
         * Construct a swing phase:
         *    Creates a 3D swing reference motion
         *    Creates a 1D clearance profile for SDF based obstacle avoidance.
         * @param liftOff: Information about the liftoff event.
         * @param touchDown: Information about the touchdown event.
         * @param SwingProfile: Settings to shape the swing profile
         * @param terrainModel: (optional) Pointer to the terrain model. Terrain model should be kept alive externall as long as the swingphase
         * object exists. Will extract SDF and obstacle information from the terrain.
         */
        SwingPhase(SwingEvent liftOff, SwingEvent touchDown, const SwingProfile &swingProfile,
                   const terrain_model::TerrainModel *terrainModel = nullptr);

        ~SwingPhase() override = default;

        bool contactFlag() const override { return false; };

        vector3_t normalDirectionInWorldFrame(ocs2::scalar_t time) const override;

        matrix3_t rotationMatrixInTerrainFrame(ocs2::scalar_t time) const override;

        vector3_t nominalFootholdLocation() const override;

        vector3_t getPositionInWorld(ocs2::scalar_t time) const override;

        vector3_t getVelocityInWorld(ocs2::scalar_t time) const override;

        vector3_t getAccelerationInWorld(ocs2::scalar_t time) const override;

        ocs2::scalar_t getMinimumFootClearance(ocs2::scalar_t time) const override;

      private:
        void setFullSwing(const SwingProfile &swingProfile,
          const terrain_model::TerrainModel *terrainModel);

        void setHalveSwing(const SwingProfile &swingProfile, 
          const terrain_model::TerrainModel *terrainModel);

        ocs2::scalar_t getScaling(ocs2::scalar_t time) const;

        SwingEvent liftOff_;
        SwingEvent touchDown_;

        std::unique_ptr<SwingSpline3d> motion_;
        std::unique_ptr<QuinticSwing> terrainClearanceMotion_;
    };
  } // namespace locomotion
} // namespace legged_locomotion_mpc
    
#endif