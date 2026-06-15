#include <gtest/gtest.h>

#include <legged_locomotion_mpc/soft_constraint/TerrainAvoidanceSoftConstraint.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(TerrainAvoidanceSoftConstraintTest, loader)
{
  const std::string constraintFilePath = meldogConfigFolder + "terrain_avoidance_soft_constraint_settings.info";

  const auto constraintSettings = loadTerrainAvoidanceSoftConstraintSettings(constraintFilePath);

  EXPECT_TRUE(constraintSettings.endEffectorBarrierSettings.mu == 0.5);
  EXPECT_TRUE(constraintSettings.endEffectorBarrierSettings.delta == 0.6);
  EXPECT_TRUE(constraintSettings.collisionLinksBarrierSettings.mu == 0.1);
  EXPECT_TRUE(constraintSettings.collisionLinksBarrierSettings.delta == 0.2);
}