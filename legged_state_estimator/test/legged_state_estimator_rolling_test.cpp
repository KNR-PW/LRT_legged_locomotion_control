#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>

#include <legged_state_estimator/legged_state_estimator.hpp>

using namespace ocs2;
using namespace legged_state_estimator;

namespace
{
  const scalar_t IMU_PERIOD = 0.002;
  const scalar_t LEG_LENGTH = 0.3;
  const scalar_t RADIUS = 0.035;

  // Two legs with revolute hip joints (same axis) and spherical feet, contact frames in sphere centers
  std::string writeUrdf(const std::string& axis)
  {
    const std::string path = (std::filesystem::temp_directory_path() / ("legged_state_estimator_rolling_test_" + axis + ".urdf")).string();
    std::ofstream file(path);
    const std::string inertial = "<inertial><mass value=\"1.0\"/><inertia ixx=\"0.1\" iyy=\"0.1\" izz=\"0.1\""
      " ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial>";
    file << "<?xml version=\"1.0\"?><robot name=\"test\">"
         << "<link name=\"base_link\">" << inertial << "</link><link name=\"imu_link\"/>"
         << "<joint name=\"imu_joint\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"imu_link\"/></joint>";
    for(const std::string leg: {"left", "right"})
    {
      file << "<link name=\"" << leg << "_leg\">" << inertial << "</link><link name=\"" << leg << "_foot\"/>"
           << "<joint name=\"" << leg << "_joint\" type=\"revolute\"><parent link=\"base_link\"/><child link=\"" << leg << "_leg\"/>"
           << "<origin xyz=\"0 " << (leg == "left" ? 0.2 : -0.2) << " 0\"/><axis xyz=\"" << axis << "\"/>"
           << "<limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
           << "<joint name=\"" << leg << "_foot_joint\" type=\"fixed\"><parent link=\"" << leg << "_leg\"/>"
           << "<child link=\"" << leg << "_foot\"/><origin xyz=\"0 0 " << -LEG_LENGTH << "\"/></joint>";
    }
    file << "</robot>";
    return path;
  }

  LeggedStateEstimatorSettings settings(scalar_t radius)
  {
    LeggedStateEstimatorSettings settings;
    settings.world_frame = "world";
    settings.base_frame = "base_link";
    settings.imu_frame = "imu_link";
    settings.contact_frames = {"left_foot", "right_foot"};
    if(radius >= 0.0) settings.contact_sphere_radii = {radius, radius};
    settings.contact_position_noise = 0.001;
    settings.contact_rotation_noise = 0.01;
    settings.sampling_time = IMU_PERIOD;
    settings.lpf_gyro_accel_cutoff_frequency = 200.0;
    settings.lpf_lin_accel_cutoff_frequency = 200.0;
    settings.lpf_dqJ_cutoff_frequency = 30.0;
    settings.lpf_ddqJ_cutoff_frequency = 30.0;
    settings.lpf_tauJ_cutoff_frequency = 30.0;
    settings.initial_gyroscope_bias_noise = 0.001;
    settings.initial_accelerometer_bias_noise = 0.1;
    // Kinematics is trusted much more than accelerometer, so estimate follows the contact model
    settings.inekf_noise_params.setGyroscopeNoise(0.01);
    settings.inekf_noise_params.setGyroscopeBiasNoise(0.0001);
    settings.inekf_noise_params.setAccelerometerNoise(1.0);
    settings.inekf_noise_params.setAccelerometerBiasNoise(0.001);
    settings.inekf_noise_params.setContactNoise(0.01);
    return settings;
  }

  struct Result
  {
    vector3_t positionError;
    vector3_t velocityError;
    vector3_t exactDisplacement;
  };

  /**
   * Legs rotate by angle(t) about axis (base does not rotate), feet roll without slipping on flat ground.
   * IMU measures the exact base motion. Errors are at the end of motion (position) and in its middle (velocity).
   */
  Result roll(const std::string& axis, const vector3_t& axisVector, scalar_t modelRadius)
  {
    const scalar_t duration = 0.5;
    const scalar_t amplitude = 0.4;
    const auto angle = [&](scalar_t t) {return amplitude * (t / duration - std::sin(2.0 * M_PI * t / duration) / (2.0 * M_PI));};
    const auto angleRate = [&](scalar_t t) {return amplitude / duration * (1.0 - std::cos(2.0 * M_PI * t / duration));};

    // Sphere center moves by rotation x (r * n), base is the center minus rotated leg
    const auto basePosition = [&](scalar_t t)
    {
      const scalar_t theta = angle(t);
      const vector3_t center = vector3_t(0.0, 0.0, RADIUS) + (theta * axisVector).cross(vector3_t(0.0, 0.0, RADIUS));
      return vector3_t(center - Eigen::AngleAxisd(theta, axisVector).toRotationMatrix() * vector3_t(0.0, 0.0, -LEG_LENGTH));
    };
    const auto baseVelocity = [&](scalar_t t) {const scalar_t h = 1e-5; return vector3_t((basePosition(t + h) - basePosition(t - h)) / (2.0 * h));};
    const auto baseAcceleration = [&](scalar_t t)
    {
      const scalar_t h = 1e-4; return vector3_t((basePosition(t + h) - 2.0 * basePosition(t) + basePosition(t - h)) / (h * h));
    };

    LeggedStateEstimator estimator(writeUrdf(axis), settings(modelRadius));
    estimator.init(basePosition(0.0), vector4_t(0.0, 0.0, 0.0, 1.0));

    Result result;
    const size_t samples = static_cast<size_t>(duration / IMU_PERIOD);
    for(size_t k = 0; k <= samples; ++k)
    {
      const scalar_t t = k * IMU_PERIOD;
      const vector3_t specificForce = baseAcceleration(t) + vector3_t(0.0, 0.0, PLUS_GRAVITY_VALUE);
      estimator.updateImu(vector3_t::Zero(), specificForce, t);
      if(k == 0) estimator.updateContacts({{0, true}, {1, true}}, t);
      estimator.correct(vector_t::Constant(2, angle(t)), vector_t::Constant(2, angleRate(t)), vector_t::Zero(2), t);
      if(k == samples / 2) result.velocityError = estimator.getBaseLinearVelocityEstimateWorld() - baseVelocity(t);
    }
    result.positionError = estimator.getBasePositionEstimate() - basePosition(duration);
    result.exactDisplacement = basePosition(duration) - basePosition(0.0);
    return result;
  }
} // namespace

TEST(LeggedStateEstimatorRollingTest, rollingIsCompensatedAboutBothHorizontalAxes)
{
  for(const auto& [axis, axisVector]: {std::pair<std::string, vector3_t>{"0 1 0", vector3_t::UnitY()},
    std::pair<std::string, vector3_t>{"1 0 0", vector3_t::UnitX()}})
  {
    const Result point = roll(axis, axisVector, 0.0);
    const Result sphere = roll(axis, axisVector, RADIUS);

    // Base moves by (leg length + radius) * angle, point contact model sees only leg length * angle
    const scalar_t rolling = RADIUS * 0.4;
    EXPECT_GT(point.positionError.norm(), 0.5 * rolling) << "axis " << axis;
    EXPECT_LT(sphere.positionError.norm(), 0.1 * rolling) << "axis " << axis << " error " << sphere.positionError.transpose();
    EXPECT_LT(sphere.velocityError.norm(), 0.25 * point.velocityError.norm()) << "axis " << axis;
    EXPECT_GT(sphere.exactDisplacement.head<2>().norm(), 0.1);
  }
}

TEST(LeggedStateEstimatorRollingTest, zeroRadiusIsPointContact)
{
  // Empty radii and zero radii are the same (default: point or surface contacts)
  const Result defaultRadii = roll("0 1 0", vector3_t::UnitY(), -1.0);
  const Result zeroRadii = roll("0 1 0", vector3_t::UnitY(), 0.0);
  EXPECT_LT((defaultRadii.positionError - zeroRadii.positionError).norm(), 1e-12);
}

TEST(LeggedStateEstimatorRollingTest, wrongRadii)
{
  const std::string urdf = writeUrdf("0 1 0");
  auto negative = settings(-1.0); negative.contact_sphere_radii = {0.03, -0.01};
  EXPECT_THROW(LeggedStateEstimator(urdf, negative), std::invalid_argument);
  auto notFinite = settings(-1.0); notFinite.contact_sphere_radii = {0.03, std::numeric_limits<scalar_t>::quiet_NaN()};
  EXPECT_THROW(LeggedStateEstimator(urdf, notFinite), std::invalid_argument);
  auto wrongSize = settings(-1.0); wrongSize.contact_sphere_radii = {0.03};
  EXPECT_THROW(LeggedStateEstimator(urdf, wrongSize), std::invalid_argument);
}

TEST(LeggedStateEstimatorRollingTest, radiiLoader)
{
  const auto load = [](const std::string& radii)
  {
    const std::string path = (std::filesystem::temp_directory_path() / "legged_state_estimator_rolling_test.info").string();
    std::ofstream file(path);
    file << "legged_state_estimator\n{\n  contactFrames\n  {\n    [0] left_foot\n    [1] right_foot\n  }\n" << radii << "}\n";
    file.close();
    return loadLeggedStateEstimatorSettings(path, "legged_state_estimator", false);
  };

  EXPECT_TRUE(load("").contact_sphere_radii.empty());
  const auto loaded = load("  contactSphereRadii\n  {\n    [0] 0.035\n    [1] 0.0\n  }\n");
  ASSERT_EQ(loaded.contact_sphere_radii.size(), 2);
  EXPECT_EQ(loaded.contact_sphere_radii[0], 0.035);
  EXPECT_EQ(loaded.contact_sphere_radii[1], 0.0);
  EXPECT_THROW(load("  contactSphereRadii\n  {\n    [0] 0.035\n  }\n"), std::invalid_argument);
}
