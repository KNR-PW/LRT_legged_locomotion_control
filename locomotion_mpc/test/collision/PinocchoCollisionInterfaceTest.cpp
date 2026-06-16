#include <gtest/gtest.h>

#include "../include/definitions.hpp"

#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/collision/CollisionSettings.hpp>

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::collision;

const scalar_t tolerance = 1e-5;

const size_t NUM_TEST = 20;

TEST(PinocchoCollisionInterfaceTest, getters)
{
  const std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    meldog3DofContactNames, meldog6DofContactNames);

  ModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  modelSettings.endEffectorSixDofNames = {};

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.terrainRelaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.terrainCollisionLinkNames.size(), 0.5);
  collisionSettings.selfCollisionRelaxations = {0.5, 0.5};
  collisionSettings.shrinkRatio = 0.5;

  PinocchioCollisionInterface collisionInterface(modelInfo, modelSettings, 
    collisionSettings, interface);

  const auto terrainindices = collisionInterface.getTerrainAvoidanceCollisionIndices();
  
  const std::vector<size_t> trueTerrainindices{4, 5};
  EXPECT_TRUE(terrainindices == trueTerrainindices);

  const auto selfCollisionindices = collisionInterface.getSelfCollisionIndices();

  EXPECT_TRUE(selfCollisionindices[0].first == 4);
  EXPECT_TRUE(selfCollisionindices[0].second == 5);
  EXPECT_TRUE(selfCollisionindices[1].first == 4);
  EXPECT_TRUE(selfCollisionindices[1].second == 6);
  EXPECT_TRUE(selfCollisionindices[2].first == 1);
  EXPECT_TRUE(selfCollisionindices[2].second == 7);
  

  std::vector<scalar_t> circleRadius(8);

  // End effectors
  for(size_t i = 0; i < 4; ++i)
  {
    circleRadius[i] = 0.1;
  }

  // Collision
  circleRadius[4] = 0.1;
  circleRadius[5] = 1.0;
  circleRadius[6] = 0.5;
  circleRadius[7] = 0.1875;

  std::vector<vector3_t> circlePositon(8);

  // End effectors
  for(size_t i = 0; i < 4; ++i)
  {
    circlePositon[i] = {0.0, 0.0, 0.0};
  }
  // Collision
  circlePositon[4] = {0.0, 0.0, 0.0};
  circlePositon[5] = {1.0, 0.0, 0.0};
  circlePositon[6] = {1.0, 1.0, 1.0};
  circlePositon[7] = {0.804653, 2.2102, 2.5162};

  std::vector<size_t> circleNumber(8);

  // End effectors
  for(size_t i = 0; i < 4; ++i)
  {
    circleNumber[i] = 1;
  }
  // Collision
  circleNumber[4] = 1;
  circleNumber[5] = 1;
  circleNumber[6] = 1;
  circleNumber[7] = 4;

  std::vector<std::vector<std::vector<size_t>>> collisionNeighbours(8);

  // End effectors
  std::vector<std::vector<size_t>> endEffectorNeighbours(1);
  endEffectorNeighbours[0] = std::vector<size_t>{0};
  for(size_t i = 0; i < 7; ++i)
  {
    collisionNeighbours[i] = endEffectorNeighbours; 
  }

  std::vector<std::vector<size_t>> seventhCollisionNeighbours(4);
  seventhCollisionNeighbours[0] = std::vector<size_t>{0, 1, 2, 3};
  seventhCollisionNeighbours[1] = std::vector<size_t>{1, 0, 2, 3};
  seventhCollisionNeighbours[2] = std::vector<size_t>{2, 3, 1, 0};
  seventhCollisionNeighbours[3] = std::vector<size_t>{3, 2, 1, 0};
  collisionNeighbours[7] = seventhCollisionNeighbours;

  for(size_t i = 0; i < 8; ++i)
  {
    EXPECT_TRUE(circleNumber[i] == collisionInterface.getFrameSphereNumber(i));
    EXPECT_TRUE(std::abs(circleRadius[i] - collisionInterface.getFrameSphereRadiuses(i)[0]) < tolerance);
    EXPECT_TRUE((circlePositon[i] - collisionInterface.getFrameSpherePositions(i)[0]).norm() < tolerance);
    
    for(size_t j = 0; j < collisionNeighbours[i].size(); ++j)
    {
      const auto& neighbours = collisionInterface.getSphereNeighbours(i, j);
      EXPECT_TRUE(collisionNeighbours[i][j] == neighbours);
    }
  }
}