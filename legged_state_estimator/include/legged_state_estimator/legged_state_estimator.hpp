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
    /// @brief Constructor.
    /// @param[in] settings State estimator settings.
    ///
    LeggedStateEstimator(const std::string& urdf_path, 
      const LeggedStateEstimatorSettings& settings);

    ///
    /// @brief Default destructor.
    ///
    ~LeggedStateEstimator();

    LeggedStateEstimator(const LeggedStateEstimator&) = default;
    LeggedStateEstimator& operator=(const LeggedStateEstimator&) = default;
    LeggedStateEstimator(LeggedStateEstimator&&) noexcept = default;
    LeggedStateEstimator& operator=(LeggedStateEstimator&&) noexcept = default;

    ///
    /// @brief Initializes the state estimator.
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
      const std::vector<std::pair<int,bool>>& contacts);

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
    /// @return const reference to the conatct estimator. 
    ///
    const ContactEstimator& getContactEstimator() const;

    ///
    /// @return const reference to the state estimator settings. 
    ///
    const LeggedStateEstimatorSettings& getSettings() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    LeggedStateEstimatorSettings settings_;

    InEKF inekf_;

    vectorKinematics leg_kinematics_;

    RobotModel robot_model_;

    ContactEstimator contact_estimator_;

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
  };
} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR__