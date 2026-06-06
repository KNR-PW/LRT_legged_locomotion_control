//
// Created by rgrandia on 13.03.20.
// Modified by Bartłomiej Krajewski (://github.com/BartlomiejK2) on 01.09.2025 
//

#ifndef __SWING_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __SWING_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/SingleLegLogic.hpp>
#include <legged_locomotion_mpc/locomotion/FootPhase.hpp>
#include <legged_locomotion_mpc/locomotion/OverExtensionPenalty.hpp>

#include <terrain_model/core/ConvexTerrain.hpp>
#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class SwingTrajectoryPlanner 
    {
      public:

        using forwardKinematics = legged_locomotion_mpc::PinocchioForwardEndEffectorKinematicsCppAd;

        struct StaticSettings 
        {
          /** Velocity at the start of swing */
          ocs2::scalar_t liftOffVelocity = 0.0;

          /** Velocity at the end of swing */
          ocs2::scalar_t touchDownVelocity = 0.0;
          
          /** 
           * Proportional gain for returning to the planned swing trajectory.
           * 10-90%-rise_time ~= 2.2 / errorGain
           * Alternatively can be measured as:
           * (velocity feedback) / (tracking error) ([m/s] / [m])
           */ 
          ocs2::scalar_t errorGain = 0.0;
          
          /** Swing phases shorter than this time will be scaled down in height and velocity */ 
          ocs2::scalar_t swingTimeScale = 0.15;

          /** Desired sdf based clearance in the middle of the swing phase [m] */
          ocs2::scalar_t sdfMidswingMargin = 0.0; 

          /** Shrinkage of the convex terrain constrains in [m] */
          ocs2::scalar_t terrainMargin = 0.0; 
          
          /** Factor in [0, 1] with which to take previous foothold into account */ 
          ocs2::scalar_t previousFootholdFactor = 0.0;

          /** Previous foothold is taken if the new reference is within this threshold. [m] */
          ocs2::scalar_t previousFootholdDeadzone = 0.0;

          /** Previous foothold is taken if the contact phase is starting withing this time. [s] */
          ocs2::scalar_t previousFootholdTimeDeadzone = 0.0; 
          
          /** Leg extension beyond this length [m] will be penalized in terrain selection */
          ocs2::scalar_t nominalLegExtension = 0.35;

          /** Weight of the end effector overextension penalty */
          ocs2::scalar_t legOverExtensionWeight = 5.0; 
          
          /** 
           * Base and foot references generated for this amount of seconds
           * after the horizon ends. 
           */
          ocs2::scalar_t referenceExtensionAfterHorizon = 0.2;

          /** Limits the amount of additional swing height from terrain adaptation. */
          ocs2::scalar_t maxSwingHeightAdaptation = 0.4;
        };

        struct DynamicSettings
        {
          /** Maximum height of foots at midpoint of swing phase */
          std::vector<ocs2::scalar_t> swingHeights;

          /** Time progress for midpoints in the swing phase in [0, 1) */
          std::vector<ocs2::scalar_t> phases;

          /** Swing progress for midpoints in tangential direction in [0, 1] */
          std::vector<ocs2::scalar_t> tangentialProgresses;

          /** Tangantial velocity as a factor of the average velocity.
           *  The tangential velocity will be: velocityFactor * swingdistance / dt 
           */
          std::vector<ocs2::scalar_t> tangentialVelocityFactors;
        };

        SwingTrajectoryPlanner(floating_base_model::FloatingBaseModelInfo info,
          StaticSettings staticSettings,
          DynamicSettings initDynamicSettings,
          const forwardKinematics& forwardKinematics,
          const OverExtensionPenalty& overExtensionPenalty);

        /** update inverted pendulum foothold adjustment height (base height) */
        void updateInvertedPendulumHeight(ocs2::scalar_t invertedPendulumHeight);

        /** Update terrain model */
        void updateTerrain(const terrain_model::TerrainModel& terrainModel);
        
        /** Update dynamic settings */
        void updateDynamicSettings(const DynamicSettings& newDynamicSettings);

        /** Access the SDF of the current terrain model */ 
        const terrain_model::SignedDistanceField* getSignedDistanceField() const;

        /** 
         * Main interface preparing all swing trajectories in cartesian
         * space (called by reference manager). 
         * Target trajectories should be subsampled already.
         * */ 
        void updateSwingMotions(ocs2::scalar_t initTime,
          ocs2::scalar_t finalTime, const ocs2::SystemObservation& currentObservation,
          const ocs2::TargetTrajectories& targetTrajectories,
          const ocs2::ModeSchedule& modeSchedule);

        /** Main access method for the generated cartesian references. */ 
        const FootPhase& getFootPhase(size_t endEffectorIndex, ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference positions */
        std::vector<vector3_t> getEndEffectorPositions(ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference velocities */
        std::vector<vector3_t> getEndEffectorVelocities(ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference terrain clerance */
        std::vector<ocs2::scalar_t> getEndEffectorClearances(ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference surface normal */
        std::vector<vector3_t> getEndEffectorSurfaceNormals(ocs2::scalar_t time) const;
        
        /** 
         * Main access method for generated cartesian reference position trajectories 
         * 3D position of end effector i in time (index) t: pos[t][i]
         */
        using position_trajectories = std::vector<std::vector<vector3_t>>;
        position_trajectories getEndEffectorPositionTrajectories(
          std::vector<ocs2::scalar_t> times) const;

        /** 
         * Main access method for generated cartesian reference velocity trajectories 
         * 3D velocity of end effector i in time (index) t: vel[t][i]
         */
        using velocity_trajectories = std::vector<std::vector<vector3_t>>;
        velocity_trajectories getEndEffectorVelocityTrajectories(
          std::vector<ocs2::scalar_t> times) const;
        
        /** 
         * Main access method for generated cartesian reference terrain clearance 
         * trajectories of end effector i in time (index) t: vel[t][i]
         */
        using foot_clearance_trajectory = std::vector<std::vector<ocs2::scalar_t>>;
        foot_clearance_trajectory getEndEffectorClearanceTrajectories(
          std::vector<ocs2::scalar_t> times) const;

        /** 
         * Main access method for generated cartesian reference surface normals 
         * of end effector i in time (index) t: nrom[t][i]
         */
        using normal_trajectories = std::vector<std::vector<vector3_t>>;
        normal_trajectories getEndEffectorNormalTrajectories(
          std::vector<ocs2::scalar_t> times) const;
        
        /**
         * Complete foot trajectories for single timestamp
         */
        struct EndEffectorTrajectoriesPoint
        {
          std::vector<vector3_t> positions;
          std::vector<vector3_t> velocities;
          std::vector<vector3_t> surfaceNormals;
          std::vector<ocs2::scalar_t> clearances;
        };

        /**
         * Complete foot trajectories for all timestamps
         */
        struct EndEffectorTrajectories
        {
          position_trajectories positions;
          velocity_trajectories velocities;
          normal_trajectories surfaceNormals;
          foot_clearance_trajectory clearances;
        };

        /** Main access method for generated end effector trajectory point */
        EndEffectorTrajectoriesPoint getEndEffectorTrajectoryPoint(
          ocs2::scalar_t time) const;

        /** 
         * Main, optimized version of upper 3 algorithms for generating reference foot
         * trajectories. Access of end effector i in time (index) t: points[t] -> pos[i], vel[i], cl[i]
         */
        EndEffectorTrajectories getEndEffectorTrajectories(
          std::vector<ocs2::scalar_t> times) const;

        struct FootTangentialConstraintTrajectories
        {
          /**
           * Times where at least one constraint matrix changes 
           */
          std::vector<ocs2::scalar_t> times;
          /**
           * Vector with constraints matrixes.
           * Get constraint matrix of end effector i in time (index) t: constraints[t][i]
           */
          std::vector<std::vector<FootTangentialConstraintMatrix>> constraints;
        };

        FootTangentialConstraintTrajectories getFootTangentialConstraintTrajectories();

        /** Accessed by the controller for visualization */
        std::vector<terrain_model::ConvexTerrain> getNominalFootholds(
          size_t endEffectorIndex) const;

        /** Accessed by the controller for visualization */
        std::vector<vector3_t> getHeuristicFootholds(size_t endEffectorIndex) const;

        /** Read static settings */
        const StaticSettings& getStaticSettings() const;

        /** Read dynamic settings */
        const DynamicSettings& getDynamicSettings() const;

      private:

        void updateLastContact(size_t endEffectorIndex, ocs2::scalar_t expectedLiftOff,
          const vector3_t& currentFootPosition, const terrain_model::TerrainModel& terrainModel);
        
        using FootPhasesStamped =  std::pair<std::vector<ocs2::scalar_t>, std::vector<std::unique_ptr<FootPhase>>>; 
        FootPhasesStamped generateSwingTrajectories(size_t endEffectorIndex, 
          const std::vector<ContactTiming> &contactTimings, ocs2::scalar_t finalTime) const;

        std::vector<vector3_t> selectHeuristicFootholds(size_t endEffectorIndex,
          const std::vector<ContactTiming> &contactTimings,
          const ocs2::TargetTrajectories &targetTrajectories, ocs2::scalar_t initTime,
          const ocs2::SystemObservation& currentObservation, ocs2::scalar_t finalTime) const;

        std::vector<terrain_model::ConvexTerrain> selectNominalFootholdTerrain(
          size_t endEffectorIndex, const std::vector<ContactTiming> &contactTimings,
          const std::vector<vector3_t> &heuristicFootholds,
          const ocs2::TargetTrajectories &targetTrajectories, ocs2::scalar_t initTime,
          const ocs2::SystemObservation& currentObservation, ocs2::scalar_t finalTime,
          const terrain_model::TerrainModel &terrainModel) const;

        void applySwingMotionScaling(SwingPhase::SwingEvent &liftOff, 
          SwingPhase::SwingEvent &touchDown, SwingPhase::SwingProfile &swingProfile) const;

        SwingPhase::SwingProfile getDynamicSwingProfile(size_t endEffectorIndex) const;

        ocs2::scalar_t getContactEndTime(const ContactTiming &contactPhase,
          ocs2::scalar_t finalTime) const;

        const FootPhase* getFootPhaseIfInContact(size_t endEffectorIndex, 
          ocs2::scalar_t time) const;

        vector3_t filterFoothold(const vector3_t &newFoothold,
           const vector3_t &previousFoothold) const;

        const floating_base_model::FloatingBaseModelInfo modelInfo_;
        const StaticSettings staticSettings_;
        DynamicSettings dynamicSettings_;
        ocs2::scalar_t invertedPendulumHeight_ = -1.0;

        const forwardKinematics& forwardKinematics_;

        ocs2::ModeSchedule modeSchedule_;

        std::vector<std::pair<ocs2::scalar_t, terrain_model::TerrainPlane>> lastContacts_;
        std::vector<std::vector<std::unique_ptr<FootPhase>>> feetNormalTrajectories_;
        std::vector<std::vector<ocs2::scalar_t>> feetNormalTrajectoriesEvents_;

        std::vector<std::vector<terrain_model::ConvexTerrain>> nominalFootholdsPerLeg_;
        std::vector<std::vector<vector3_t>> heuristicFootholdsPerLeg_;

        const terrain_model::TerrainModel* terrainModel_;

        const OverExtensionPenalty overExtensionPenalty_;
    };

    /**
     * Creates SwingTrajectoryPlanner static settings 
     * @param [in] filename: file path with swing planner settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] verbose: verbose flag
     * @return SwingTrajectoryPlanner::StaticSettings struct
     */
    SwingTrajectoryPlanner::StaticSettings loadSwingPlannerStaticSettings(
      const std::string& filename, 
      const std::string& fieldName = "swing_planner_static_settings",
      bool verbose = true);

    /**
     * Creates SwingTrajectoryPlanner dynamic settings
     * @param [in] filename: file path with swing planner settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] verbose: verbose flag
     * @return SwingTrajectoryPlanner::DynamicSettings struct
     */
    SwingTrajectoryPlanner::DynamicSettings loadSwingPlannerDynamicSettings(
      const std::string& filename, 
      const floating_base_model::FloatingBaseModelInfo& info,
      const std::string& fieldName = "swing_planner_dynamic_settings",
      bool verbose = true);
  } // namespace locomotion
} //  namespace legged_locomotion_mpc

#endif