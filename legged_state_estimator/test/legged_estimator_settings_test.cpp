#include <gtest/gtest.h>

#include <legged_state_estimator/legged_state_estimator_settings.hpp>
#include <legged_state_estimator/path_management/package_path.h>

using namespace ocs2;
using namespace legged_state_estimator;

const scalar_t eps = 1e-9;

TEST(ModelSettingsTest, loader)
{
  const std::string filePath = package_path::getPath() + "/test/example_settings.info";

  const auto settings = loadLeggedStateEstimatorSettings(filePath);

  std::vector<std::string> true_contact_frames{"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  std::vector<scalar_t> true_beta0{-20.0, -20.0, -20.0, -20.0};
  std::vector<scalar_t> true_beta1{0.7, 0.7, 0.7, 0.7};

  const matrix3_t true_gyro_noise = 0.01 * 0.01 * matrix3_t::Identity();
  const matrix3_t true_gyro_bias_noise = 0.00001 * 0.00001 * matrix3_t::Identity();
  const matrix3_t true_accel_noise = 0.1 * 0.1 * matrix3_t::Identity();
  const matrix3_t true_accel_bias_noise = 0.0001 * 0.0001 * matrix3_t::Identity();
  const matrix3_t true_contact_noise = 0.1 * 0.1 * matrix3_t::Identity();

  EXPECT_TRUE(settings.imu_frame == "imu_link");
  EXPECT_TRUE(settings.contact_frames == true_contact_frames);
  EXPECT_TRUE(settings.contact_estimator_settings.beta0 == true_beta0);
  EXPECT_TRUE(settings.contact_estimator_settings.beta1 == true_beta1);
  EXPECT_TRUE(settings.contact_estimator_settings.contact_force_covariance_alpha == 100.0);
  EXPECT_TRUE(settings.contact_estimator_settings.contact_probability_threshold == 0.5);
  EXPECT_TRUE((settings.inekf_noise_params.getGyroscopeCov() - true_gyro_noise).norm() < eps);
  EXPECT_TRUE((settings.inekf_noise_params.getAccelerometerCov() - true_accel_noise).norm() < eps);
  EXPECT_TRUE((settings.inekf_noise_params.getGyroscopeBiasCov() - true_gyro_bias_noise).norm() < eps);
  EXPECT_TRUE((settings.inekf_noise_params.getAccelerometerBiasCov() - true_accel_bias_noise).norm() < eps);
  EXPECT_TRUE((settings.inekf_noise_params.getContactCov() - true_contact_noise).norm() < eps);
  EXPECT_TRUE(settings.dynamic_contact_estimation == false);
  EXPECT_TRUE(settings.contact_position_noise == 0.01);
  EXPECT_TRUE(settings.contact_rotation_noise == 0.01);
  EXPECT_TRUE(settings.sampling_time == 0.01);
  EXPECT_TRUE(settings.lpf_gyro_accel_cutoff_frequency == 250);
  EXPECT_TRUE(settings.lpf_lin_accel_cutoff_frequency  == 250);
  EXPECT_TRUE(settings.lpf_dqJ_cutoff_frequency        == 10);
  EXPECT_TRUE(settings.lpf_ddqJ_cutoff_frequency       == 5);
  EXPECT_TRUE(settings.lpf_tauJ_cutoff_frequency       == 10);
}