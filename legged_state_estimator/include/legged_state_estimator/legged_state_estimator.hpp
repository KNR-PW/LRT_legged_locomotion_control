/******************************************************************************
BSD 3-Clause License

Copyright (c) 2019, Ross Hartley
Copyright (c) 2023, mayataka
Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2026
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR__
#define __LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR__

#include <legged_state_estimator/types.hpp>
#include <legged_state_estimator/inekf/inekf.hpp>
#include <legged_state_estimator/inekf/inekf_state.hpp>
#include <legged_state_estimator/inekf/noise_params.hpp>
#include <legged_state_estimator/inekf/observations.hpp>
#include <legged_state_estimator/robot_model.hpp>
#include <legged_state_estimator/contact_estimator.hpp>
#include <legged_state_estimator/low_pass_filter.hpp>
#include <legged_state_estimator/legged_state_estimator_settings.hpp>


namespace legged_state_estimator 
{
  ///
  /// @class LeggedStateEstimator
  /// @brief State estimator for legged robots.
  ///
  class LeggedStateEstimator 
  {
    public:

    ///
    /// @brief Constructor. All base estimates and init() arguments refer to
    /// LeggedStateEstimatorSettings::base_frame. The IMU must be rigidly
    /// attached to it (only fixed joints in between), otherwise throws
    /// std::invalid_argument.
    /// @param[in] settings State estimator settings.
    ///
    LeggedStateEstimator(const std::string& urdf_path,
      const LeggedStateEstimatorSettings& settings);

    ///
    /// @brief Initializes the state estimator. Base angular velocity is assumed
    /// to be zero.
    /// @param[in] base_pos Base position.
    /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w).
    /// @param[in] base_lin_vel_world Base linear velocity expressed in the world
    /// coordinate. Default is vector3_t::Zero().
    /// @param[in] imu_gyro_bias Initial guess of the IMU gyro bias. Default is 
    /// vector3_t::Zero().
    /// @param[in] imu_lin_accel_bias Initial guess of the IMU linear acceleration 
    /// bias. Default is vector3_t::Zero().
    ///
    void init(const vector3_t& base_pos, const vector4_t& base_quat,
      const vector3_t& base_lin_vel_world=vector3_t::Zero(),
      const vector3_t& imu_gyro_bias=vector3_t::Zero(),
      const vector3_t& imu_lin_accel_bias=vector3_t::Zero());

    ///
    /// @brief Initializes the state estimator.
    /// @param[in] base_pos Base position. 
    /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
    /// @param[in] qJ Raw measurement of the joint positions. 
    /// @param[in] ground_height Ground height. 
    /// @param[in] base_lin_vel_world Base linear velocity expressed in the world
    /// coordinate. Default is vector3_t::Zero().
    /// @param[in] imu_gyro_bias Initial guess of the IMU gyro bias. Default is 
    /// vector3_t::Zero().
    /// @param[in] imu_lin_accel_bias Initial guess of the IMU linear acceleration 
    /// bias. Default is vector3_t::Zero().
    ///
    void init(const vector3_t& base_pos, const vector4_t& base_quat,
      const ocs2::vector_t& qJ, 
      const std::vector<ocs2::scalar_t>& ground_height,
      const vector3_t& base_lin_vel_world=vector3_t::Zero(),
      const vector3_t& imu_gyro_bias=vector3_t::Zero(),
      const vector3_t& imu_lin_accel_bias=vector3_t::Zero());

    ///
    /// @brief Updates the state estimation.
    /// @param[in] imu_gyro_raw Raw measurement of the base angular velocity 
    /// expressed in the body local coordinate from IMU gyro sensor.
    /// @param[in] imu_lin_accel_raw Raw measurement of the base linear 
    /// acceleration expressed in the body local coordinate from IMU accelerometer. 
    /// @param[in] qJ Raw measurement of the joint positions. 
    /// @param[in] dqJ Raw measurement of the joint velocities. 
    /// @param[in] tauJ Raw measurement of the joint torques. 
    ///
    void update(const vector3_t& imu_gyro_raw, 
      const vector3_t& imu_lin_accel_raw, 
      const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ, 
      const ocs2::vector_t& tauJ);

    ///
    /// @brief Updates the state estimation.
    /// @param[in] imu_gyro_raw Raw measurement of the base angular velocity 
    /// expressed in the body local coordinate from IMU gyro sensor.
    /// @param[in] imu_lin_accel_raw Raw measurement of the base linear 
    /// acceleration expressed in the body local coordinate from IMU accelerometer. 
    /// @param[in] qJ Raw measurement of the joint positions. 
    /// @param[in] dqJ Raw measurement of the joint velocities. 
    /// @param[in] contacts Contact states from external sensors
    ///
    void update(const vector3_t& imu_gyro_raw, 
      const vector3_t& imu_lin_accel_raw, 
      const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ, 
      const ocs2::vector_t& tauJ,
      const std::vector<std::pair<int,bool>>& contacts);

    ///
    /// @brief Updates the state estimation using the actual time since the
    /// previous update instead of LeggedStateEstimatorSettings::sampling_time.
    /// @param[in] imu_gyro_raw Raw measurement of the base angular velocity
    /// expressed in the body local coordinate from IMU gyro sensor.
    /// @param[in] imu_lin_accel_raw Raw measurement of the base linear
    /// acceleration expressed in the body local coordinate from IMU accelerometer.
    /// @param[in] qJ Raw measurement of the joint positions.
    /// @param[in] dqJ Raw measurement of the joint velocities.
    /// @param[in] tauJ Raw measurement of the joint torques.
    /// @param[in] dt Time since the previous update [s]. Must be positive.
    ///
    void update(const vector3_t& imu_gyro_raw,
      const vector3_t& imu_lin_accel_raw,
      const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
      const ocs2::vector_t& tauJ, const ocs2::scalar_t dt);

    ///
    /// @brief Updates the state estimation using the actual time since the
    /// previous update instead of LeggedStateEstimatorSettings::sampling_time.
    /// @param[in] imu_gyro_raw Raw measurement of the base angular velocity
    /// expressed in the body local coordinate from IMU gyro sensor.
    /// @param[in] imu_lin_accel_raw Raw measurement of the base linear
    /// acceleration expressed in the body local coordinate from IMU accelerometer.
    /// @param[in] qJ Raw measurement of the joint positions.
    /// @param[in] dqJ Raw measurement of the joint velocities.
    /// @param[in] contacts Contact states from external sensors
    /// @param[in] dt Time since the previous update [s]. Must be positive.
    ///
    void update(const vector3_t& imu_gyro_raw,
      const vector3_t& imu_lin_accel_raw,
      const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
      const ocs2::vector_t& tauJ,
      const std::vector<std::pair<int,bool>>& contacts,
      const ocs2::scalar_t dt);

    ///
    /// @brief Processes an IMU measurement, when IMU and joint measurements are
    /// processed separately (updateImu() and correct() instead of update(),
    /// do not mix them). The estimate is propagated to the measurement time
    /// with the previous IMU measurement held constant, then the new measurement
    /// is held until the next measurement. Times are in seconds, relative to
    /// any start time (absolute ROS time in double resolves only ~0.24 us).
    /// @param[in] imu_gyro_raw Raw measurement of the base angular velocity
    /// expressed in the body local coordinate from IMU gyro sensor.
    /// @param[in] imu_lin_accel_raw Raw measurement of the base linear
    /// acceleration expressed in the body local coordinate from IMU accelerometer.
    /// @param[in] time Measurement time [s]. Must increase between IMU measurements.
    ///
    void updateImu(const vector3_t& imu_gyro_raw,
      const vector3_t& imu_lin_accel_raw, const ocs2::scalar_t time);

    ///
    /// @brief Sets contact states from external sensors (use_contact_estimator
    /// is false), used by the following correct() calls.
    /// @param[in] contacts Contact states (contact id from 0 to number of contacts - 1,
    /// in contact). Ids must be valid and unique, contacts that are not given keep their state.
    /// @param[in] time Measurement time [s]. Must not be older than the previous contact states.
    ///
    void updateContacts(const std::vector<std::pair<int,bool>>& contacts,
      const ocs2::scalar_t time);

    ///
    /// @brief Corrects the state estimation with joint measurements. Contact
    /// states come from the contact estimator or from the last updateContacts()
    /// call (required after init() when use_contact_estimator is false).
    /// The estimate is first propagated to the measurement time with the last
    /// IMU measurement held constant. Measurement older than the estimate is
    /// used at the estimate time.
    /// @param[in] qJ Raw measurement of the joint positions.
    /// @param[in] dqJ Raw measurement of the joint velocities.
    /// @param[in] tauJ Raw measurement of the joint torques.
    /// @param[in] time Measurement time [s]. Must increase between joint measurements.
    ///
    void correct(const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
      const ocs2::vector_t& tauJ, const ocs2::scalar_t time);

    ///
    /// @return Time of the state estimate [s]: newest measurement time given
    /// to updateImu() or correct() since init().
    ///
    ocs2::scalar_t getEstimateTime() const;

    ///
    /// @return const reference to the base position estimate.
    ///
    const vector3_t& getBasePositionEstimate() const;

    ///
    /// @return const reference to the base orientation estimate expressed by a 
    /// rotation matrix.
    ///
    const matrix3_t& getBaseRotationEstimate() const;

    ///
    /// @return const reference to the base orientation estimate expressed by 
    /// quaternion.
    ///
    const vector4_t& getBaseQuaternionEstimate() const;

    ///
    /// @return const reference to the base linear velocity estimate expressed in 
    /// the world frame.
    ///
    const vector3_t& getBaseLinearVelocityEstimateWorld() const;

    ///
    /// @return const reference to the base linear velocity estimate expressed in 
    /// the body local coordinate.
    ///
    const vector3_t getBaseLinearVelocityEstimateLocal() const;

    ///
    /// @return const reference to the base angular velocity estimate expressed in 
    /// the world frame.
    ///
    const vector3_t& getBaseAngularVelocityEstimateWorld() const;

    ///
    /// @return const reference to the base angular velocity estimate expressed in 
    /// the local frame.
    ///
    const vector3_t& getBaseAngularVelocityEstimateLocal() const;

    ///
    /// @return const reference to the IMU gyro bias estimate. 
    ///
    const vector3_t& getIMUGyroBiasEstimate() const;

    ///
    /// @return const reference to the IMU linear acceleration bias estimate. 
    ///
    const vector3_t& getIMULinearAccelerationBiasEstimate() const;

    ///
    /// @return const reference to the joint velocity estimates. 
    ///
    const ocs2::vector_t& getJointVelocityEstimate() const;

    ///
    /// @return const reference to the joint acceleration estimates. 
    ///
    const ocs2::vector_t& getJointAccelerationEstimate() const;

    ///
    /// @return const reference to the joint torque estimates. 
    ///
    const ocs2::vector_t& getJointTorqueEstimate() const;

    ///
    /// @return const reference to the conatct estimator. Throws
    /// std::logic_error if use_contact_estimator is false.
    ///
    const ContactEstimator& getContactEstimator() const;

    ///
    /// @return const reference to the state estimator settings. 
    ///
    const LeggedStateEstimatorSettings& getSettings() const;

    ///
    /// @return const reference to robot model 
    ///
    const RobotModel& getRobotModel() const;

    ///
    /// @brief Sets the ground normals at the contacts, used by the contact
    /// estimator to compute normal contact forces. Default is empty (flat ground).
    /// @param[in] ground_normals Ground normal for each contact expressed in
    /// the world frame (normalized internally). Size must be number of contacts,
    /// or 0 for flat ground (world z axis at every contact).
    ///
    void setGroundNormals(const std::vector<vector3_t>& ground_normals);

    ///
    /// @return const reference to the normalized ground normals expressed in
    /// the world frame. Empty means flat ground.
    ///
    const std::vector<vector3_t>& getGroundNormals() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    void checkUpdateArguments(const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
      const ocs2::vector_t& tauJ, const ocs2::scalar_t dt) const;

    void setImuMeasurement(const vector3_t& imu_gyro_raw,
      const vector3_t& imu_lin_accel_raw);

    ocs2::scalar_t prepareCorrection(const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
      const ocs2::vector_t& tauJ, const ocs2::scalar_t time);

    void propagateTo(const ocs2::scalar_t time);

    void processImuMeasurement(const ocs2::scalar_t dt);

    void processJointMeasurements(const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
      const ocs2::vector_t& tauJ, const ocs2::scalar_t dt);

    void correctKinematics();

    void restoreEstimates();

    LeggedStateEstimatorSettings settings_;

    InEKF inekf_;

    vectorKinematics leg_kinematics_;

    RobotModel robot_model_;

    std::unique_ptr<ContactEstimator> contact_estimator_ptr_;

    LowPassFilter<ocs2::scalar_t, 3> lpf_gyro_accel_world_;
    LowPassFilter<ocs2::scalar_t, 3> lpf_lin_accel_world_;
    
    LowPassFilter<ocs2::scalar_t, Eigen::Dynamic> lpf_dqJ_;
    LowPassFilter<ocs2::scalar_t, Eigen::Dynamic> lpf_ddqJ_;
    LowPassFilter<ocs2::scalar_t, Eigen::Dynamic> lpf_tauJ_;
     
    matrix3_t base_rot_estimate_;
    vector6_t imu_raw_;
    vector4_t base_quat_estimate_;

    vector3_t imu_gyro_raw_world_;
    vector3_t imu_gyro_raw_world_prev_;

    vector3_t imu_gyro_accel_world_;
    vector3_t imu_gyro_accel_local_;

    vector3_t imu_lin_accel_raw_world_;
    vector3_t imu_lin_accel_local_;

    vector3_t base_pos_estimate_;

    vector3_t base_lin_vel_world_estimate_;
    vector3_t base_lin_vel_local_estimate_;

    vector3_t base_ang_vel_world_estimate_;
    vector3_t base_ang_vel_local_estimate_;

    vector3_t imu_gyro_bias_estimate_;
    vector3_t imu_lin_acc_bias_estimate_;

    bool imu_filters_initialized_;
    bool joint_filters_initialized_;
    bool imu_measurement_available_;
    bool joint_measurement_available_;
    bool estimate_time_available_;
    ocs2::scalar_t estimate_time_;
    ocs2::scalar_t last_imu_time_;
    ocs2::scalar_t last_joint_time_;
    bool contacts_available_;
    ocs2::scalar_t last_contacts_time_;

    std::vector<vector3_t> ground_normals_;
  };
} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR__