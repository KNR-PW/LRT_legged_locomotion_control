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
      /// @brief Name of the base frame specified in the URDF file.
      ///
      std::string base_frame;

      /// 
      /// @brief Name of the IMU frame specified in the URDF file.
      ///
      std::string imu_frame;

      /// 
      /// @brief Names of the contact frames specified in the URDF file.
      ///
      std::vector<std::string> contact_frames;

      ///
      /// @brief Flag if contact estimator is used, or contact flags are given by external system
      ///
      bool use_contact_estimator;

      /// 
      /// @brief Contact estimator settings. 
      ///
      ContactEstimatorSettings contact_estimator_settings;

      /// 
      /// @brief Noise parameters (covariances) of InEKF. 
      ///
      NoiseParams inekf_noise_params;

      /// 
      /// @brief Use dynamics in contact estimation. If false, equilibrium is 
      /// used for contact estimation. Default is false.
      ///
      bool dynamic_contact_estimation = false;

      /// 
      /// @brief Noise (covariance) on contact position. (Possibly is not used in 
      /// InEKF. Contact covariance in noise_params are more important).
      ///
      ocs2::scalar_t contact_position_noise;

      /// 
      /// @brief Noise (covariance) on contact rotation. Only used with surface 
      /// contacts.
      ///
      ocs2::scalar_t contact_rotation_noise;

      /// 
      /// @brief Time step of estimation. 
      ///
      ocs2::scalar_t sampling_time;

      /// 
      /// @brief Cutoff frequency of LPF for gyro sensor. 
      ///
      ocs2::scalar_t lpf_gyro_cutoff_frequency;

      /// 
      /// @brief Cutoff frequency of LPF for gyro acceleration that is computed by
      /// finite difference approximation. 
      ///
      ocs2::scalar_t lpf_gyro_accel_cutoff_frequency;

      /// 
      /// @brief Cutoff frequency of LPF for linear acceleration measurement from IMU. 
      ///
      ocs2::scalar_t lpf_lin_accel_cutoff_frequency;

      /// 
      /// @brief Cutoff frequency of LPF for joint velocities. 
      ///
      ocs2::scalar_t lpf_dqJ_cutoff_frequency;

      /// 
      /// @brief Cutoff frequency of LPF for joint accelerations. 
      ///
      ocs2::scalar_t lpf_ddqJ_cutoff_frequency;

      /// 
      /// @brief Cutoff frequency of LPF for joint torques. 
      ///
      ocs2::scalar_t lpf_tauJ_cutoff_frequency;
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