#include "legged_state_estimator/legged_state_estimator_settings.hpp"


namespace legged_state_estimator {

LeggedStateEstimatorSettings LeggedStateEstimatorSettings::UnitreeA1(
    const std::string& urdf_path, const double sampling_time) {
  LeggedStateEstimatorSettings settings;
  settings.urdf_path = urdf_path;
  settings.imu_frame = "imu_link";
  settings.contact_frames = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"}; 

  settings.contact_estimator_settings.beta0 = {-20.0, -20.0, -20.0, -20.0};
  settings.contact_estimator_settings.beta1 = {0.7, 0.7, 0.7, 0.7};
  settings.contact_estimator_settings.contact_force_covariance_alpha = 100.0;
  settings.contact_estimator_settings.contact_probability_threshold = 0.5;

  settings.inekf_noise_params.setGyroscopeNoise(0.01);
  settings.inekf_noise_params.setAccelerometerNoise(0.1);
  settings.inekf_noise_params.setGyroscopeBiasNoise(0.00001);
  settings.inekf_noise_params.setAccelerometerBiasNoise(0.0001);
  settings.inekf_noise_params.setContactNoise(0.1);

  settings.dynamic_contact_estimation = false;

  settings.contact_position_noise = 0.01;
  settings.contact_rotation_noise = 0.01;

  settings.sampling_time = sampling_time;

  settings.lpf_gyro_accel_cutoff_frequency = 250;
  settings.lpf_lin_accel_cutoff_frequency  = 250;
  settings.lpf_dqJ_cutoff_frequency        = 10;
  settings.lpf_ddqJ_cutoff_frequency       = 5;
  settings.lpf_tauJ_cutoff_frequency       = 10;

  return settings;
}

} // namespace legged_state_estimator
