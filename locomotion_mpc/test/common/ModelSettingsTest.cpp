#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(ModelSettingsTest, loader)
{
  const std::string filePath = meldogConfigFolder + "model_settings.info";

  const auto modelSettings = loadModelSettings(filePath);

  EXPECT_TRUE(modelSettings.robotName == "TEST_ROBOT");
  EXPECT_TRUE(modelSettings.algorithm == "SQP");
  EXPECT_TRUE(modelSettings.verbose == false);
  EXPECT_TRUE(modelSettings.recompileLibrariesCppAd == false);
  EXPECT_TRUE(modelSettings.modelFolderCppAd == "/tmp/legged_locomotion_mpc");
  EXPECT_TRUE(modelSettings.worldLinkName == "base_link");
  EXPECT_TRUE(modelSettings.baseLinkName == "trunk_link");
  EXPECT_TRUE(modelSettings.endEffectorThreeDofNames == meldog3DofContactNames);
  EXPECT_TRUE(modelSettings.endEffectorSixDofNames.size() == 0);
  EXPECT_TRUE(modelSettings.hipFrameNames == meldogHipNames);
}