#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include <legged_state_estimator/legged_state_estimator.hpp>

using namespace ocs2;
using namespace legged_state_estimator;

namespace
{
  const scalar_t IMU_PERIOD = 0.002;

  // Order of joint measurements and IMU measurements
  enum class JointOrder
  {
    AFTER_IMU,         // Same time as IMU, processed after it
    BEFORE_IMU,        // Same time as IMU, processed before it
    BETWEEN_IMU,       // Between times of two IMU measurements
    NEWER_THAN_LATE_IMU // Newer than IMU measurement, but processed before it
  };

  LeggedStateEstimator makeEstimator(scalar_t initialGyroBiasNoise = 1.0, scalar_t initialAccelBiasNoise = 1.0)
  {
    const std::string path = (std::filesystem::temp_directory_path() / "legged_state_estimator_imu_test.urdf").string();
    std::ofstream file(path);
    const std::string inertial = "<inertial><mass value=\"1.0\"/><inertia ixx=\"0.1\" iyy=\"0.1\" izz=\"0.1\""
      " ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial>";
    file << "<?xml version=\"1.0\"?><robot name=\"test\">"
         << "<link name=\"base_link\">" << inertial << "</link>"
         << "<link name=\"imu_link\"/>"
         << "<joint name=\"imu_joint\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"imu_link\"/></joint>";
    for(const std::string leg: {"left", "right"})
    {
      file << "<link name=\"" << leg << "_foot\">" << inertial << "</link>"
           << "<joint name=\"" << leg << "_joint\" type=\"revolute\"><parent link=\"base_link\"/>"
           << "<child link=\"" << leg << "_foot\"/><origin xyz=\"0 " << (leg == "left" ? 0.2 : -0.2) << " -0.3\"/><axis xyz=\"0 1 0\"/>"
           << "<limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>";
    }
    file << "</robot>";
    file.close();

    LeggedStateEstimatorSettings settings;
    settings.world_frame = "world";
    settings.base_frame = "base_link";
    settings.imu_frame = "imu_link";
    settings.contact_frames = {"left_foot", "right_foot"};
    settings.use_contact_estimator = false;
    settings.contact_position_noise = 0.01;
    settings.contact_rotation_noise = 0.01;
    settings.sampling_time = IMU_PERIOD;
    settings.lpf_gyro_accel_cutoff_frequency = 200.0;
    settings.lpf_lin_accel_cutoff_frequency = 200.0;
    settings.lpf_dqJ_cutoff_frequency = 30.0;
    settings.lpf_ddqJ_cutoff_frequency = 30.0;
    settings.lpf_tauJ_cutoff_frequency = 30.0;
    settings.initial_gyroscope_bias_noise = initialGyroBiasNoise;
    settings.initial_accelerometer_bias_noise = initialAccelBiasNoise;
    settings.inekf_noise_params.setGyroscopeNoise(0.01);
    settings.inekf_noise_params.setGyroscopeBiasNoise(0.0001);
    settings.inekf_noise_params.setAccelerometerNoise(0.1);
    settings.inekf_noise_params.setAccelerometerBiasNoise(0.001);
    settings.inekf_noise_params.setContactNoise(0.1);
    return LeggedStateEstimator(path, settings);
  }

  struct Measurements
  {
    // With jumps, as at lift off and touchdown
    static scalar_t accelerationX(size_t k) {return (k % 7 < 3) ? 5.0 : -2.0;}
    static scalar_t accelerationZ(size_t k) {return (k % 10 < 5) ? 20.0 : 0.0;}
    static scalar_t yawRate(size_t k) {return (k % 8 < 4) ? 1.5 : -0.5;}
  };

  // No contacts: estimate comes only from IMU
  LeggedStateEstimator run(JointOrder order, size_t samples, bool rotating)
  {
    LeggedStateEstimator estimator = makeEstimator();
    estimator.init(vector3_t::Zero(), vector4_t(0.0, 0.0, 0.0, 1.0));

    const auto imu = [&](size_t k)
    {
      const vector3_t gyro(0.0, 0.0, rotating ? Measurements::yawRate(k) : 0.0);
      const vector3_t acceleration(rotating ? 0.0 : Measurements::accelerationX(k), 0.0, Measurements::accelerationZ(k));
      estimator.updateImu(gyro, acceleration, k * IMU_PERIOD);
    };
    const auto joints = [&](scalar_t time)
    {
      estimator.correct(vector_t::Zero(2), vector_t::Zero(2), vector_t::Zero(2), time);
    };

    imu(0);
    estimator.updateContacts({{0, false}, {1, false}}, 0.0);

    for(size_t k = 1; k < samples; ++k)
    {
      const scalar_t time = k * IMU_PERIOD;
      switch(order)
      {
        case JointOrder::AFTER_IMU:
          imu(k);
          joints(time);
          break;
        case JointOrder::BEFORE_IMU:
          joints(time);
          imu(k);
          break;
        case JointOrder::BETWEEN_IMU:
          joints(time - 0.3 * IMU_PERIOD);
          imu(k);
          break;
        case JointOrder::NEWER_THAN_LATE_IMU:
          joints(time + 0.4 * IMU_PERIOD);
          imu(k);
          break;
      }
    }
    return estimator;
  }

  const std::vector<JointOrder> ALL_ORDERS{JointOrder::AFTER_IMU, JointOrder::BEFORE_IMU,
    JointOrder::BETWEEN_IMU, JointOrder::NEWER_THAN_LATE_IMU};
} // namespace

TEST(LeggedStateEstimatorImuTest, velocityIsTrapezoidForEveryOrder)
{
  const size_t samples = 104;
  for(const JointOrder order: ALL_ORDERS)
  {
    const auto estimator = run(order, samples, false);

    vector3_t velocity = vector3_t::Zero();
    for(size_t k = 1; k < samples; ++k)
    {
      velocity.x() += 0.5 * (Measurements::accelerationX(k - 1) + Measurements::accelerationX(k)) * IMU_PERIOD;
      velocity.z() += (0.5 * (Measurements::accelerationZ(k - 1) + Measurements::accelerationZ(k))
        + MINUS_GRAVITY_VALUE) * IMU_PERIOD;
    }

    // Estimate is newer than the last IMU measurement, it is held
    const scalar_t held = (order == JointOrder::NEWER_THAN_LATE_IMU) ? 0.4 * IMU_PERIOD : 0.0;
    velocity.x() += Measurements::accelerationX(samples - 1) * held;
    velocity.z() += (Measurements::accelerationZ(samples - 1) + MINUS_GRAVITY_VALUE) * held;

    EXPECT_NEAR(estimator.getEstimateTime(), (samples - 1) * IMU_PERIOD + held, 1e-12);
    EXPECT_LT((estimator.getBaseLinearVelocityEstimateWorld() - velocity).norm(), 1e-9)
      << "order " << static_cast<int>(order);
  }
}

TEST(LeggedStateEstimatorImuTest, orientationIsTrapezoidForEveryOrder)
{
  const size_t samples = 103;
  for(const JointOrder order: ALL_ORDERS)
  {
    const auto estimator = run(order, samples, true);

    scalar_t yaw = 0.0;
    for(size_t k = 1; k < samples; ++k)
    {
      yaw += 0.5 * (Measurements::yawRate(k - 1) + Measurements::yawRate(k)) * IMU_PERIOD;
    }
    if(order == JointOrder::NEWER_THAN_LATE_IMU)
    {
      yaw += Measurements::yawRate(samples - 1) * 0.4 * IMU_PERIOD;
    }

    const matrix3_t& rotation = estimator.getBaseRotationEstimate();
    EXPECT_NEAR(std::atan2(rotation(1, 0), rotation(0, 0)), yaw, 1e-9) << "order " << static_cast<int>(order);
    EXPECT_LT((rotation * rotation.transpose() - matrix3_t::Identity()).norm(), 1e-9);
  }
}

TEST(LeggedStateEstimatorImuTest, positionDoesNotDependOnOrder)
{
  const size_t samples = 104;
  const vector3_t position = run(JointOrder::AFTER_IMU, samples, false).getBasePositionEstimate();

  // Same estimate time, position differs only by the shape of acceleration in one time step
  for(const JointOrder order: {JointOrder::BEFORE_IMU, JointOrder::BETWEEN_IMU})
  {
    EXPECT_LT((run(order, samples, false).getBasePositionEstimate() - position).norm(), 1e-4)
      << "order " << static_cast<int>(order);
  }
}

// Base turns, but feet in contact do not move relative to it (wrong kinematics, as rolling or slipping feet).
// Initial bias is right, so gyroscope should be trusted
TEST(LeggedStateEstimatorImuTest, wrongKinematicsDoesNotChangeKnownGyroBias)
{
  const scalar_t yawRate = 0.5;

  const auto turn = [&](scalar_t initialGyroBiasNoise)
  {
    LeggedStateEstimator estimator = makeEstimator(initialGyroBiasNoise, 0.1);
    estimator.init(vector3_t(0.0, 0.0, 0.3), vector4_t(0.0, 0.0, 0.0, 1.0));
    estimator.updateImu(vector3_t(0.0, 0.0, yawRate), vector3_t(0.0, 0.0, PLUS_GRAVITY_VALUE), 0.0);
    estimator.updateContacts({{0, true}, {1, true}}, 0.0);
    for(size_t k = 1; k <= 500; ++k)
    {
      estimator.updateImu(vector3_t(0.0, 0.0, yawRate), vector3_t(0.0, 0.0, PLUS_GRAVITY_VALUE), k * IMU_PERIOD);
      estimator.correct(vector_t::Zero(2), vector_t::Zero(2), vector_t::Zero(2), k * IMU_PERIOD);
    }
    return estimator.getIMUGyroBiasEstimate().z();
  };

  // Uncertain bias takes most of the yaw rate, known one stays
  EXPECT_GT(turn(1.0), 0.5 * yawRate);
  EXPECT_LT(std::abs(turn(0.001)), 0.02 * yawRate);
}

TEST(LeggedStateEstimatorImuTest, wrongInitialBiasNoise)
{
  EXPECT_THROW(makeEstimator(0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(makeEstimator(-1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(makeEstimator(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(makeEstimator(1.0, std::nan("")), std::invalid_argument);
}
