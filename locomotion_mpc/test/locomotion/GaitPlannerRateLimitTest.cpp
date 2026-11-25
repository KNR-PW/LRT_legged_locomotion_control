#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

namespace
{
  // MPC iteration and gait update offset, as in LeggedReferenceManager
  const scalar_t MPC_PERIOD = 0.025;
  const scalar_t UPDATE_OFFSET = 0.1;
  const scalar_t HORIZON = 1.0;

  GaitStaticParameters staticParameters(scalar_t frequencyRate, scalar_t swingRatioRate)
  {
    GaitStaticParameters params;
    params.endEffectorNumber = 4;
    params.initPhase = 0.0;
    params.maximumSteppingFrequency = 3.0;
    params.minimumSteppingFrequency = 0.0;
    params.touchdownWindow = 0.1;
    params.maximumSteppingFrequencyRate = frequencyRate;
    params.maximumSwingRatioRate = swingRatioRate;
    return params;
  }

  // Trot: end effectors 0 and 3 together, 1 and 2 half period later
  GaitDynamicParameters trot(scalar_t frequency, scalar_t swingRatio)
  {
    GaitDynamicParameters params;
    params.steppingFrequency = frequency;
    params.swingRatio = swingRatio;
    params.phaseOffsets = {0.5, 0.5, 0.0};
    return params;
  }

  // Contact flags of mode schedule equal to flags of the gait planner (away from mode changes)
  void expectConsistentModeSchedule(GaitPlanner& planner, const ModeSchedule& modeSchedule,
    scalar_t startTime, scalar_t finalTime)
  {
    const auto& eventTimes = modeSchedule.eventTimes;
    for(scalar_t time = startTime + 1e-3; time < finalTime; time += 0.0137)
    {
      const bool nearEvent = std::any_of(eventTimes.begin(), eventTimes.end(),
        [time](scalar_t eventTime) {return std::abs(eventTime - time) < 2e-3;});
      if(nearEvent) continue;

      EXPECT_EQ(contact_flags_t(modeSchedule.modeAtTime(time)), planner.getContactFlagsAtTime(time))
        << "time " << time;
    }
  }

  /**
   * One MPC iteration: step of dynamic parameters at initTime + UPDATE_OFFSET, then mode schedule.
   * Checks that the step does not change phases and contact flags at its time.
   */
  ModeSchedule mpcIteration(GaitPlanner& planner, scalar_t initTime,
    const GaitDynamicParameters* newTarget = nullptr)
  {
    const scalar_t stepTime = initTime + UPDATE_OFFSET;
    const auto phasesBefore = planner.getPhasesAtTime(stepTime);
    const auto flagsBefore = planner.getContactFlagsAtTime(stepTime);

    if(newTarget)
    {
      planner.updateDynamicParameters(stepTime, *newTarget);
    }
    else
    {
      planner.stepDynamicParameters(stepTime);
    }

    const auto phasesAfter = planner.getPhasesAtTime(stepTime);
    for(size_t i = 0; i < phasesBefore.size(); ++i)
    {
      EXPECT_NEAR(phasesBefore[i], phasesAfter[i], 1e-9) << "time " << stepTime;
    }
    EXPECT_EQ(flagsBefore, planner.getContactFlagsAtTime(stepTime)) << "time " << stepTime;

    const auto modeSchedule = planner.getModeSchedule(initTime, initTime + HORIZON);
    expectConsistentModeSchedule(planner, modeSchedule, initTime, initTime + HORIZON);
    return modeSchedule;
  }
} // namespace

TEST(GaitPlannerRateLimitTest, steppingFrequencyIsRateLimited)
{
  const scalar_t frequencyRate = 2.0;
  GaitPlanner planner(staticParameters(frequencyRate, 0.0), trot(1.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  const auto target = trot(2.0, 0.4);
  scalar_t initTime = 0.5;
  mpcIteration(planner, initTime, &target);

  // First step is limited by MAX_PARAMETERS_STEP_DURATION
  EXPECT_NEAR(planner.getDynamicParameters().steppingFrequency,
    1.0 + frequencyRate * Definitions::MAX_PARAMETERS_STEP_DURATION, 1e-9);
  EXPECT_EQ(planner.getTargetDynamicParameters(), target);

  scalar_t previousFrequency = planner.getDynamicParameters().steppingFrequency;
  for(size_t k = 0; k < 40; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);

    const scalar_t frequency = planner.getDynamicParameters().steppingFrequency;
    EXPECT_GE(frequency, previousFrequency - 1e-12);
    EXPECT_LE(frequency - previousFrequency, frequencyRate * MPC_PERIOD + 1e-9);
    previousFrequency = frequency;
  }
  // 0.8 Hz left after the first step needs 0.4 s
  EXPECT_NEAR(planner.getDynamicParameters().steppingFrequency, 2.0, 1e-9);
  EXPECT_EQ(planner.getDynamicParameters(), target);
}

TEST(GaitPlannerRateLimitTest, swingRatioIncreaseDoesNotLiftOffAgain)
{
  // No rate limit, only contacts limit the change
  GaitPlanner planner(staticParameters(0.0, 0.0), trot(1.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // At step time 0.45 end effectors 0 and 3 (phase 0.45) touched down 0.05 s ago, 1 and 2 (phase 0.95) in stance
  const auto target = trot(1.0, 0.6);
  scalar_t initTime = 0.35;
  mpcIteration(planner, initTime, &target);

  // Swing ratio is stopped at the phase of end effectors 0 and 3, they stay in contact
  EXPECT_NEAR(planner.getDynamicParameters().swingRatio, 0.45, 1e-6);
  EXPECT_EQ(planner.getContactFlagsAtTime(0.45), contact_flags_t(15));

  // Swing ratio follows them until target
  scalar_t previousSwingRatio = planner.getDynamicParameters().swingRatio;
  for(size_t k = 0; k < 20; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
    const scalar_t swingRatio = planner.getDynamicParameters().swingRatio;
    EXPECT_GE(swingRatio, previousSwingRatio - 1e-12);
    previousSwingRatio = swingRatio;
  }
  EXPECT_NEAR(planner.getDynamicParameters().swingRatio, 0.6, 1e-9);
}

TEST(GaitPlannerRateLimitTest, swingRatioDecreaseDoesNotTouchDownInTheAir)
{
  const scalar_t swingRatioRate = 0.5;
  GaitPlanner planner(staticParameters(0.0, swingRatioRate), trot(2.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // At step time 0.15 end effectors 0 and 3 are in swing (phase 0.3), touchdown at phase 0.4
  const auto target = trot(2.0, 0.2);
  scalar_t initTime = 0.05;
  mpcIteration(planner, initTime, &target);
  EXPECT_EQ(planner.getContactFlagsAtTime(0.15), contact_flags_t(6));

  scalar_t previousSwingRatio = planner.getDynamicParameters().swingRatio;
  EXPECT_LT(previousSwingRatio, 0.4);
  for(size_t k = 0; k < 40; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
    const scalar_t swingRatio = planner.getDynamicParameters().swingRatio;
    EXPECT_LE(swingRatio, previousSwingRatio + 1e-12);
    // Step after a blocked one is longer (average rate is kept), but not longer than MAX_PARAMETERS_STEP_DURATION
    EXPECT_LE(previousSwingRatio - swingRatio, swingRatioRate * Definitions::MAX_PARAMETERS_STEP_DURATION + 1e-9);
    previousSwingRatio = swingRatio;
  }
  EXPECT_NEAR(planner.getDynamicParameters().swingRatio, 0.2, 1e-9);
}

TEST(GaitPlannerRateLimitTest, standingRobotDoesNotLiftEndEffectors)
{
  // Standing: phases do not change (init phase 0.5 for all end effectors, swing ratio 0.25 -> all in contact)
  GaitStaticParameters params = staticParameters(2.0, 0.0);
  params.initPhase = 0.5;
  GaitDynamicParameters standing;
  standing.steppingFrequency = 0.0;
  standing.swingRatio = 0.25;
  standing.phaseOffsets = {0.0, 0.0, 0.0};

  GaitPlanner planner(params, standing, 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Swing ratio above the phases while standing would lift all end effectors, new base phase
  // (above swing ratio) keeps them in contact
  auto standingWithBigSwingRatio = standing;
  standingWithBigSwingRatio.swingRatio = 0.7;
  scalar_t initTime = 0.2;
  mpcIteration(planner, initTime, &standingWithBigSwingRatio);
  EXPECT_EQ(planner.getDynamicParameters(), standingWithBigSwingRatio);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 0.5), contact_flags_t(15));

  // Start is not rate limited, it starts with all end effectors in contact
  auto walking = standingWithBigSwingRatio;
  walking.steppingFrequency = 2.0;
  initTime += MPC_PERIOD;
  mpcIteration(planner, initTime, &walking);
  EXPECT_EQ(planner.getDynamicParameters(), walking);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 1e-3), contact_flags_t(15));

  for(size_t k = 0; k < 40; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
  }
  EXPECT_EQ(planner.getDynamicParameters(), walking);

  // Stop is not rate limited, robot stands on all end effectors
  initTime += MPC_PERIOD;
  mpcIteration(planner, initTime, &standingWithBigSwingRatio);
  EXPECT_NEAR(planner.getDynamicParameters().steppingFrequency, 0.0, 1e-9);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 0.5), contact_flags_t(15));
}

TEST(GaitPlannerRateLimitTest, phaseOffsetsChangeIsApplied)
{
  // Gait type change is not limited, with all other parameters
  GaitPlanner planner(staticParameters(0.5, 0.1), trot(1.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  GaitDynamicParameters pace = trot(2.0, 0.3);
  pace.phaseOffsets = {0.5, 0.0, 0.5};
  planner.updateDynamicParameters(0.3, pace);
  EXPECT_EQ(planner.getDynamicParameters(), pace);

  const auto modeSchedule = planner.getModeSchedule(0.25, 0.25 + HORIZON);
  expectConsistentModeSchedule(planner, modeSchedule, 0.25, 0.25 + HORIZON);
}

TEST(GaitPlannerRateLimitTest, newTargetDuringRamp)
{
  const scalar_t frequencyRate = 1.0;
  GaitPlanner planner(staticParameters(frequencyRate, 0.0), trot(1.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  const auto fast = trot(2.0, 0.4);
  scalar_t initTime = 0.0;
  mpcIteration(planner, initTime, &fast);
  for(size_t k = 0; k < 4; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
  }
  const scalar_t frequency = planner.getDynamicParameters().steppingFrequency;
  EXPECT_GT(frequency, 1.0);
  EXPECT_LT(frequency, 2.0);

  // Ramp goes back from the current frequency
  const auto slow = trot(1.05, 0.4);
  initTime += MPC_PERIOD;
  mpcIteration(planner, initTime, &slow);
  EXPECT_NEAR(planner.getDynamicParameters().steppingFrequency, frequency - frequencyRate * MPC_PERIOD, 1e-9);
  for(size_t k = 0; k < 20; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
  }
  EXPECT_EQ(planner.getDynamicParameters(), slow);
}

TEST(GaitPlannerRateLimitTest, staticParametersLoader)
{
  ModelSettings modelSettings;
  modelSettings.endEffectorThreeDofNames = {"a", "b", "c", "d"};

  const auto load = [&](const std::string& rates)
  {
    const std::string path = (std::filesystem::temp_directory_path() / "gait_rate_limit_test.info").string();
    std::ofstream file(path);
    file << "gait_static_settings\n{\n  initPhase 0.5\n  maximumSteppingFrequency 2.5\n"
         << "  minimumSteppingFrequency 0.0\n  touchdownWindow 0.75\n" << rates << "}\n";
    file.close();
    return loadGaitStaticParameters(path, modelSettings, "gait_static_settings", false);
  };

  // Optional, no limit when not set
  const auto noLimit = load("");
  EXPECT_EQ(noLimit.maximumSteppingFrequencyRate, 0.0);
  EXPECT_EQ(noLimit.maximumSwingRatioRate, 0.0);

  const auto limited = load("  maximumSteppingFrequencyRate 2.0\n  maximumSwingRatioRate 0.5\n");
  EXPECT_EQ(limited.maximumSteppingFrequencyRate, 2.0);
  EXPECT_EQ(limited.maximumSwingRatioRate, 0.5);

  EXPECT_THROW(load("  maximumSteppingFrequencyRate -1.0\n"), std::invalid_argument);
  // Too small to change parameters by GAIT_PARAMETERS_EPS in MAX_PARAMETERS_STEP_DURATION
  EXPECT_THROW(load("  maximumSwingRatioRate 0.001\n"), std::invalid_argument);
}
