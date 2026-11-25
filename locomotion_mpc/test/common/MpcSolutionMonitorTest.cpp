#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

#include <legged_locomotion_mpc/common/MpcSolutionMonitor.hpp>

using namespace legged_locomotion_mpc;
using namespace ocs2;

namespace
{
  using Result = MpcSolutionMonitor::Result;

  PerformanceIndex index(scalar_t dynamicsViolation, scalar_t cost = 100.0)
  {
    PerformanceIndex performanceIndex;
    performanceIndex.cost = cost;
    performanceIndex.dynamicsViolationSSE = dynamicsViolation;
    return performanceIndex;
  }
} // namespace

TEST(MpcSolutionMonitorTest, validSolutions)
{
  MpcSolutionMonitor monitor(0.05);
  // Walking: high cost and temporary dynamics violation are not divergence
  for(const scalar_t violation: {0.0, 1e-12, 1e-4, 1e-2, 3.5e-2, 0.05})
  {
    EXPECT_EQ(monitor.update(index(violation, 2e6)), Result::VALID);
    EXPECT_EQ(monitor.getDivergedSolutions(), 0);
  }
}

// Dynamics violations from simulation with diverged MPC
TEST(MpcSolutionMonitorTest, divergedSolutions)
{
  MpcSolutionMonitor monitor(0.05);
  for(const scalar_t violation: {3.8e-5, 1.9e-5, 1.3e-2})
  {
    EXPECT_EQ(monitor.update(index(violation)), Result::VALID);
  }

  EXPECT_EQ(monitor.update(index(0.685)), Result::DIVERGED_RESET);
  EXPECT_EQ(monitor.getDivergedSolutions(), 1);

  // Solver was reset, next solutions get better
  EXPECT_EQ(monitor.update(index(0.3)), Result::DIVERGED);
  EXPECT_EQ(monitor.update(index(0.1)), Result::DIVERGED);
  EXPECT_EQ(monitor.getDivergedSolutions(), 3);
  EXPECT_EQ(monitor.update(index(1e-3)), Result::VALID);
  EXPECT_EQ(monitor.getDivergedSolutions(), 0);

  // Next divergence resets the solver at once
  EXPECT_EQ(monitor.update(index(10.0)), Result::DIVERGED_RESET);
}

TEST(MpcSolutionMonitorTest, resetIsRepeated)
{
  MpcSolutionMonitor monitor(0.05, 3);
  const std::vector<Result> results{Result::DIVERGED_RESET, Result::DIVERGED, Result::DIVERGED};
  for(size_t i = 0; i < 12; ++i)
  {
    EXPECT_EQ(monitor.update(index(1.0)), results[i % 3]) << i;
  }

  monitor.reset();
  EXPECT_EQ(monitor.getDivergedSolutions(), 0);
  EXPECT_EQ(monitor.update(index(1.0)), Result::DIVERGED_RESET);
}

TEST(MpcSolutionMonitorTest, notFiniteSolutions)
{
  const scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
  const scalar_t infinity = std::numeric_limits<scalar_t>::infinity();

  // Solver is always reset
  MpcSolutionMonitor monitor(0.05);
  for(size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(monitor.update(index(nan)), Result::DIVERGED_RESET);
    EXPECT_EQ(monitor.update(index(0.0, infinity)), Result::DIVERGED_RESET);
    EXPECT_EQ(monitor.update(index(infinity)), Result::DIVERGED_RESET);
  }

  auto constraints = index(0.0);
  constraints.equalityConstraintsSSE = nan;
  EXPECT_EQ(monitor.update(constraints), Result::DIVERGED_RESET);
  EXPECT_EQ(monitor.update(index(0.0)), Result::VALID);
}

TEST(MpcSolutionMonitorTest, disabled)
{
  for(auto monitor: {MpcSolutionMonitor(), MpcSolutionMonitor(0.0)})
  {
    EXPECT_EQ(monitor.update(index(1e6)), Result::VALID);
    EXPECT_EQ(monitor.update(index(std::numeric_limits<scalar_t>::quiet_NaN())), Result::VALID);
  }
}

TEST(MpcSolutionMonitorTest, wrongArguments)
{
  EXPECT_THROW(MpcSolutionMonitor(-1.0), std::invalid_argument);
  EXPECT_THROW(MpcSolutionMonitor(std::numeric_limits<scalar_t>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(MpcSolutionMonitor(0.05, 0), std::invalid_argument);
}
