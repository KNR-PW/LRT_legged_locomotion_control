//
// Created by rgrandia on 29.04.20.
//

#include <gtest/gtest.h>

#include <terrain_model/core/ConvexTerrain.hpp>
#include "Rotations.hpp"

using namespace ocs2;
using namespace terrain_model;

TerrainPlane getRandomTerrain() {
  vector3_t eulerXYZ = vector3_t::Random();
  return TerrainPlane(vector3_t::Random(), rotationMatrixBaseToOrigin(eulerXYZ));
}

std::vector<vector2_t> getRandomboundary(size_t N) {
  std::vector<vector2_t> boundary(N);
  for (auto& b : boundary) {
    b.setRandom();
  }
  return boundary;
}


TEST(TestConvexTerrain, projectToConvex2dPolygonBoundary_signTest) {
  constexpr size_t N = 3;
  const auto boundary = getRandomboundary(N);

  for (size_t numTests = 0; numTests < 3; numTests++) {
    // a point inside
    const auto insidePoint = [&]() -> vector2_t {
      vector_t lambda = 0.5 * vector_t::Random(N) + 0.5 * vector_t::Ones(N);  // positive numbers in [0, 1]
      lambda /= lambda.sum();

      vector2_t p = vector2_t::Zero();
      for (size_t i = 0; i < boundary.size(); i++) {
        p += lambda(i) * boundary[i];
      }
      return p;
    }();

    // a point outside
    const auto outsidePoint = [&]() -> vector2_t {
      const vector_t lambda = vector_t::Random(N) + 2.0 * vector_t::Ones(N);  // numbers in [1, 3]

      vector2_t p = vector2_t::Zero();
      for (size_t i = 0; i < boundary.size(); i++) {
        p += lambda(i) * boundary[i];
      }
      return p;
    }();

    // a point on the polygon boundary
    const auto onPoint = [&]() -> vector2_t {
      vector_t lambda = 0.5 * vector_t::Random(2) + 0.5 * vector_t::Ones(2);  // positive numbers in [0, 1]
      lambda /= lambda.sum();
      return lambda(0) * boundary[0] + lambda(1) * boundary[1];
    }();

    ConvexTerrain terrain(getRandomTerrain(), boundary);
    EXPECT_NEAR(terrain.projectToConvex2dPolygonBoundary(onPoint).first, 0.0, 1e-9);
    EXPECT_LT(terrain.projectToConvex2dPolygonBoundary(insidePoint).first, 0.0);
    EXPECT_GT(terrain.projectToConvex2dPolygonBoundary(outsidePoint).first, 0.0);
  }
}

TEST(TestConvexTerrain, projectToConvex2dPolygonBoundary_valueTest) {
  const std::vector<vector2_t> boundary{vector2_t(-1.0, -1.0), vector2_t(-1.0, 1.0), vector2_t(1.0, 1.0), vector2_t(1.0, -1.0)};

  for (size_t numTests = 0; numTests < 3; numTests++) {
    // a point inside
    const vector2_t insidePoint = vector2_t::Random();

    const scalar_array_t insidePointDists{-std::abs(insidePoint.x() - 1.0), -std::abs(insidePoint.x() + 1.0),
                                          -std::abs(insidePoint.y() - 1.0), -std::abs(insidePoint.y() + 1.0)};
    const auto minDist = std::max_element(insidePointDists.cbegin(), insidePointDists.cend());
    
    ConvexTerrain terrain(getRandomTerrain(), boundary);
    const auto distance2ImagePair = terrain.projectToConvex2dPolygonBoundary(insidePoint);
    const auto computedDist = (distance2ImagePair.first > 0.0) ? std::sqrt(distance2ImagePair.first) : -std::sqrt(-distance2ImagePair.first);

    EXPECT_NEAR(computedDist, *minDist, 1e-9);
  }
}

TEST(TestConvexTerrain, projectToConvex2dPolygonBoundary_obtuseAngles) {
  // Regular 16-gon has obtuse angles, like convex regions from region growing
  constexpr size_t N = 16;
  const scalar_t radius = 0.1;
  std::vector<vector2_t> counterClockwise;
  for (size_t i = 0; i < N; i++) {
    const scalar_t angle = 2.0 * M_PI * i / N;
    counterClockwise.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
  }
  const std::vector<vector2_t> clockwise(counterClockwise.rbegin(), counterClockwise.rend());

  for (const auto& boundary : {counterClockwise, clockwise}) {
    ConvexTerrain terrain(TerrainPlane(), boundary);
    // Points inside of inscribed circle, close to vertices and to edges
    const scalar_t inscribedRadius = radius * std::cos(M_PI / N);
    for (size_t i = 0; i < 4 * N; i++) {
      const scalar_t angle = 2.0 * M_PI * i / (4 * N);
      const vector2_t insidePoint = 0.99 * inscribedRadius * vector2_t(std::cos(angle), std::sin(angle));
      const vector2_t outsidePoint = 1.01 * radius * vector2_t(std::cos(angle), std::sin(angle));
      EXPECT_LT(terrain.projectToConvex2dPolygonBoundary(insidePoint).first, 0.0);
      EXPECT_GT(terrain.projectToConvex2dPolygonBoundary(outsidePoint).first, 0.0);

      // Inside points are not moved by projection
      const vector3_t image = terrain.projectToConvex3dPolygon({insidePoint.x(), insidePoint.y(), 0.0});
      EXPECT_TRUE(image.head<2>().isApprox(insidePoint, 1e-9));
    }
  }
}

TEST(TestConvexTerrain, projectToConvexPolygon) {
  
  TerrainPlane plane = getRandomTerrain();
  std::vector<vector2_t> boundary = {vector2_t(-0.5, -0.5), vector2_t(-0.5, 0.5), vector2_t(0.5, 0.5), vector2_t(0.5, -0.5)};
  ConvexTerrain convexTerrain(plane, boundary);

  // plane center point
  {
    const vector3_t point = plane.getPosition();
    const vector3_t image = convexTerrain.projectToConvex3dPolygon(point);
    const vector3_t local_image = plane.getPositionInTerrainFrameFromPositionInWorld(image);

    EXPECT_NEAR(local_image.x(), 0.0, 1e-9);
    EXPECT_NEAR(local_image.y(), 0.0, 1e-9);
    EXPECT_NEAR(local_image.z(), 0.0, 1e-9);
  }

  // random point
  {
    const vector3_t point = vector3_t::Random();
    const vector3_t image = convexTerrain.projectToConvex3dPolygon(point);
    const vector3_t local_image = plane.getPositionInTerrainFrameFromPositionInWorld(image);

    EXPECT_LT(std::abs(local_image.x()), 0.5 + 1e-9);
    EXPECT_LT(std::abs(local_image.y()), 0.5 + 1e-9);
    EXPECT_NEAR(local_image.z(), 0.0, 1e-9);
  }
}
