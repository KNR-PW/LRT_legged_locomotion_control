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

#ifndef __LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR_SETTINGS__
#define __LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR_SETTINGS__

#include <legged_state_estimator/types.hpp>
#include <legged_state_estimator/inekf/noise_params.hpp>
#include <legged_state_estimator/contact_estimator.hpp>

namespace legged_state_estimator 
{

  ///
  /// @class LeggedStateEstimatorSettings
  /// @brief Settings of the legged state estimator.
  ///
  struct LeggedStateEstimatorSettings 
  {
    public:
      
      /// 
      /// @brief Name of the world frame, that robot positions etc. are relative to.
      ///
      std::string world_frame;

      ///
      /// @brief Name of the base frame specified in the URDF file. Estimates are
      /// given for this frame. It must be connected to the URDF root link only by
      /// fixed joints. If empty, the URDF root link is used.
      ///
      std::string base_frame;

      ///
      /// @brief Name of the IMU frame specified in the URDF file. It must be
      /// connected to the base frame only by fixed joints.
      ///
      std::string imu_frame;

      /// 
      /// @brief Names of the contact frames specified in the URDF file.
      ///
      std::vector<std::string> contact_frames;

      ///
      /// @brief Radii of spherical end effectors [m], in order of contact_frames. Contact frame has to be
      /// in the center of the sphere. In contact the sphere rolls without slipping, so its center moves
      /// (rotation x radius * ground normal), the estimator compensates it. 0.0: point or surface (6 DoF)
      /// contact, the contact frame does not move in contact. Empty: all 0.0 (optional in settings file).
      ///
      std::vector<ocs2::scalar_t> contact_sphere_radii;

      ///
      /// @brief Flag if contact estimator is used, or contact flags are given by external system
      ///
      bool use_contact_estimator = false;

      /// 
      /// @brief Contact estimator settings. 
      ///
      ContactEstimatorSettings contact_estimator_settings;

      /// 
      /// @brief Noise parameters (covariances) of InEKF. 
      ///
      NoiseParams inekf_noise_params;

      ///
      /// @brief Initial uncertainty (standard deviation) of the gyroscope bias [rad/s],
      /// see LeggedStateEstimator::init(). Must be positive. Should be as small as the
      /// accuracy of the initial bias: big value lets wrong kinematics (e.g. rolling or
      /// slipping feet) change the bias instead of the orientation, then yaw is wrong.
      /// Optional in settings file, default is 1.0.
      ///
      ocs2::scalar_t initial_gyroscope_bias_noise = 1.0;

      ///
      /// @brief Initial uncertainty (standard deviation) of the accelerometer bias [m/s^2],
      /// see LeggedStateEstimator::init(). Must be positive. Initial bias from calibration
      /// contains also gravity times the error of initial roll and pitch.
      /// Optional in settings file, default is 1.0.
      ///
      ocs2::scalar_t initial_accelerometer_bias_noise = 1.0;

      /// 
      /// @brief Use dynamics in contact estimation. If false, equilibrium is 
      /// used for contact estimation. Default is false.
      ///
      bool dynamic_contact_estimation = false;

      ///
      /// @brief Noise (standard deviation) on contact position measured by leg
      /// kinematics. Its square is added to the contact force covariance from
      /// the contact estimator.
      ///
      ocs2::scalar_t contact_position_noise = 0.0;

      ///
      /// @brief Noise (standard deviation) on contact rotation. Only used with surface
      /// contacts.
      ///
      ocs2::scalar_t contact_rotation_noise = 0.0;

      ///
      /// @brief Time step of estimation. Must be positive (zero default is
      /// rejected by the estimator).
      ///
      ocs2::scalar_t sampling_time = 0.0;


      ///
      /// @brief Cutoff frequency of LPF for gyro acceleration that is computed by
      /// finite difference approximation. Must be positive (zero default is
      /// rejected by the filter).
      ///
      ocs2::scalar_t lpf_gyro_accel_cutoff_frequency = 0.0;

      ///
      /// @brief Cutoff frequency of LPF for linear acceleration measurement from IMU.
      /// Must be positive (zero default is rejected by the filter).
      ///
      ocs2::scalar_t lpf_lin_accel_cutoff_frequency = 0.0;

      ///
      /// @brief Cutoff frequency of LPF for joint velocities. Must be positive
      /// (zero default is rejected by the filter).
      ///
      ocs2::scalar_t lpf_dqJ_cutoff_frequency = 0.0;

      ///
      /// @brief Cutoff frequency of LPF for joint accelerations. Must be positive
      /// (zero default is rejected by the filter).
      ///
      ocs2::scalar_t lpf_ddqJ_cutoff_frequency = 0.0;

      ///
      /// @brief Cutoff frequency of LPF for joint torques. Must be positive
      /// (zero default is rejected by the filter).
      ///
      ocs2::scalar_t lpf_tauJ_cutoff_frequency = 0.0;
  };

  /**
   * Creates Legged State Estimator Settings 
   * @param [in] filename: file path with estimator settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return LeggedStateEstimatorSettings struct
   */
  LeggedStateEstimatorSettings loadLeggedStateEstimatorSettings(const std::string& filename,
    const std::string& fieldName = "legged_state_estimator",
    bool verbose = true);
} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR_SETTINGS__