#include <gtest/gtest.h>

#include <legged_whole_body_control/WbcBase.hpp>
#include <legged_whole_body_control/WeightedWbc.hpp>
#include <legged_whole_body_control/path_management/package_path.h>

using namespace ocs2;
using namespace legged_whole_body_control;

const static scalar_t eps = 1e-9;

TEST(WbcBaseSettingsTest, loader)
{
  const std::string filePath = legged_whole_body_control::package_path::getPath() 
    + "/test/config/wbc_base.info";

  const auto settings = loadWbcBaseSettings(filePath);
  
  EXPECT_TRUE(settings.worldLinkName == "world");

  EXPECT_TRUE(settings.baseLinkName == "trunk_link");

  EXPECT_TRUE(settings.endEffectorThreeDofNames[0] == "RFF_link");
  EXPECT_TRUE(settings.endEffectorThreeDofNames[1] == "RRF_link");
  
  EXPECT_TRUE(settings.endEffectorSixDofNames[0] == "LFF_link");
  EXPECT_TRUE(settings.endEffectorSixDofNames[1] == "LRF_link");

  EXPECT_TRUE(settings.useDynamicsTask == true);
  EXPECT_TRUE(settings.useBaseTrackingTask == true);
  EXPECT_TRUE(settings.useEndEffectorsTrackingTask == true);
  EXPECT_TRUE(settings.useContactForceTrackingTask == true);
  EXPECT_TRUE(settings.useTorqueLimitsTask == true);
  EXPECT_TRUE(settings.useKinematicContactTask == false);
  EXPECT_TRUE(settings.useFrictionConeTask == true);

  const vector3_t trueBaseLinearFeedForward     = vector3_t{0.1, 0.2, 0.3};
  const vector3_t trueBaseLinearProportional    = vector3_t{0.4, 0.5, 0.6};
  const vector3_t trueBaseLinearDerivative      = vector3_t{0.7, 0.8, 0.9};
  const vector3_t trueBaseAngularFeedForward    = vector3_t{1.0, 1.1, 1.2};
  const vector3_t trueBaseAngularProportional   = vector3_t{1.3, 1.4, 1.5};
  const vector3_t trueBaseAngularDerivative     = vector3_t{1.6, 1.7, 1.8};

  EXPECT_TRUE((settings.baseSettings.linearFeedForwardGain 
    - trueBaseLinearFeedForward).norm() < eps);
  EXPECT_TRUE((settings.baseSettings.linearProportionalGain 
    - trueBaseLinearProportional).norm() < eps);
  EXPECT_TRUE((settings.baseSettings.linearDerivativeGain 
    - trueBaseLinearDerivative).norm() < eps);
  EXPECT_TRUE((settings.baseSettings.angularFeedForwardGain 
    - trueBaseAngularFeedForward).norm() < eps);
  EXPECT_TRUE((settings.baseSettings.angularProportionalGain 
    - trueBaseAngularProportional).norm() < eps);
  EXPECT_TRUE((settings.baseSettings.angularDerivativeGain 
    - trueBaseAngularDerivative).norm() < eps);

  for(size_t i = 0; i < 4; ++i)
  {
    const vector3_t trueEELinearFeedForward  = vector3_t{scalar_t(i) / 10.0, 
      scalar_t(i + 1) / 10.0, scalar_t(i + 2) / 10.0};
    const vector3_t trueEELinearProportional = vector3_t{scalar_t(i + 3) / 10.0, 
      scalar_t(i + 4) / 10.0, scalar_t(i + 5) / 10.0};
    const vector3_t trueEELinearDerivative   = vector3_t{scalar_t(i + 6) / 10.0, 
      scalar_t(i + 7) / 10.0, scalar_t(i + 8) / 10.0};

    EXPECT_TRUE((settings.endEffectorSettings.linearFeedForwardGain[i] 
      - trueEELinearFeedForward).norm() < eps);
    EXPECT_TRUE((settings.endEffectorSettings.linearProportionalGain[i] 
      - trueEELinearProportional).norm() < eps);
    EXPECT_TRUE((settings.endEffectorSettings.linearDerivativeGain[i] 
      - trueEELinearDerivative).norm() < eps);
  }
  
  for(size_t i = 0; i < 2; ++i)
  {
    const vector3_t trueEEAngularFeedForward  = vector3_t{scalar_t(i) / 10.0, 
      scalar_t(i + 1) / 10.0, scalar_t(i + 2) / 10.0};
    const vector3_t trueEEAngularProportional = vector3_t{scalar_t(i + 3) / 10.0, 
      scalar_t(i + 4) / 10.0, scalar_t(i + 5) / 10.0};
    const vector3_t trueEEAngularDerivative   = vector3_t{scalar_t(i + 6) / 10.0, 
      scalar_t(i + 7) / 10.0, scalar_t(i + 8) / 10.0};

    EXPECT_TRUE((settings.endEffectorSettings.angularFeedForwardGain[i] 
      - trueEEAngularFeedForward).norm() < eps);
    EXPECT_TRUE((settings.endEffectorSettings.angularProportionalGain[i] 
      - trueEEAngularProportional).norm() < eps);
    EXPECT_TRUE((settings.endEffectorSettings.angularDerivativeGain[i] 
      - trueEEAngularDerivative).norm() < eps);
  }

  const scalar_t trueFrictionCoefficiient = 0.6;
  EXPECT_TRUE(std::abs(settings.frictionConeSettings.frictionCoefficient 
    - trueFrictionCoefficiient) < eps);

  const scalar_t trueWbcDesiredFrequency = 100.0;
  EXPECT_TRUE(std::abs(settings.desiredFrequency 
    - trueWbcDesiredFrequency) < eps);
}

TEST(WeightedWbcSettingsTest, loader)
{
  const std::string filePath = legged_whole_body_control::package_path::getPath() 
    + "/test/config/weighted_wbc.info";

  const auto weights = loadWeightedWbcSettings(filePath);

  const scalar_t trueWeightBaseTrackingTask = 1.0;
  const scalar_t trueWeightEndEffectorsTrackingTask = 2.0;
  const scalar_t trueWeightContactForceTrackingTask = 3.0;

  EXPECT_TRUE(std::abs(weights.weightBaseTrackingTask 
    - trueWeightBaseTrackingTask) < eps);
  EXPECT_TRUE(std::abs(weights.weightEndEffectorsTrackingTask 
    - trueWeightEndEffectorsTrackingTask) < eps);
  EXPECT_TRUE(std::abs(weights.weightContactForceTrackingTask 
    - trueWeightContactForceTrackingTask) < eps);
}