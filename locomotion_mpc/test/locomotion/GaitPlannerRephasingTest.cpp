#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/GaitDynamicPhaseController.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

namespace
{
  // MPC iteration and gait update offset, as in LeggedReferenceManager and Meldog config
  const scalar_t MPC_PERIOD = 0.025;
  const scalar_t UPDATE_OFFSET = 0.25;
  const scalar_t HORIZON = 1.0;
  const contact_flags_t ALL_IN_CONTACT(15);

  // As in legged_model.info of Meldog
  GaitStaticParameters staticParameters()
  {
    GaitStaticParameters params;
    params.endEffectorNumber = 4;
    params.initPhase = 0.5;
    params.maximumSteppingFrequency = 2.5;
    params.minimumSteppingFrequency = 0.0;
    params.touchdownWindow = 0.75;
    params.maximumSteppingFrequencyRate = 2.0;
    params.maximumSwingRatioRate = 0.5;
    return params;
  }

  // Initial gait of Meldog config, all end effectors in contact (phase 0.5 above swing ratio)
  GaitDynamicParameters standing(scalar_t swingRatio = 0.25)
  {
    GaitDynamicParameters params;
    params.steppingFrequency = 0.0;
    params.swingRatio = swingRatio;
    params.phaseOffsets = {0.0, 0.0, 0.0};
    return params;
  }

  // Trot: end effectors 0 and 3 together (mode 9 in their stance), 1 and 2 half period later (mode 6)
  GaitDynamicParameters trot(scalar_t frequency, scalar_t swingRatio)
  {
    GaitDynamicParameters params;
    params.steppingFrequency = frequency;
    params.swingRatio = swingRatio;
    params.phaseOffsets = {-0.5, -0.5, 0.0};
    return params;
  }

  // Pace: end effectors 0 and 2 together, 1 and 3 half period later
  GaitDynamicParameters pace(scalar_t frequency, scalar_t swingRatio)
  {
    auto params = trot(frequency, swingRatio);
    params.phaseOffsets = {0.5, 0.0, 0.5};
    return params;
  }

  // Contact flags of mode schedule equal to flags of the gait planner (away from mode changes),
  // no too short modes
  void expectConsistentModeSchedule(GaitPlanner& planner, const ModeSchedule& modeSchedule,
    scalar_t startTime, scalar_t finalTime)
  {
    const auto& eventTimes = modeSchedule.eventTimes;
    for(scalar_t time = startTime + 1e-3; time < finalTime; time += 0.0071)
    {
      const bool nearEvent = std::any_of(eventTimes.begin(), eventTimes.end(),
        [time](scalar_t eventTime) {return std::abs(eventTime - time) < 2e-3;});
      if(nearEvent) continue;

      EXPECT_EQ(contact_flags_t(modeSchedule.modeAtTime(time)), planner.getContactFlagsAtTime(time))
        << "time " << time;
    }

    for(size_t i = 1; i < eventTimes.size(); ++i)
    {
      EXPECT_GT(eventTimes[i] - eventTimes[i - 1], Definitions::MIN_TIME_BETWEEN_CHANGES - 1e-9)
        << "event time " << eventTimes[i];
    }
  }

  /**
   * One MPC iteration: step of dynamic parameters at initTime + UPDATE_OFFSET, then mode schedule.
   * Checks that the step does not change phases and contact flags at its time (already planned ones).
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

  size_t changedContacts(const contact_flags_t& lhs, const contact_flags_t& rhs)
  {
    return (lhs ^ rhs).count();
  }
} // namespace

TEST(GaitPlannerRephasingTest, phaseControllerWithNewPhase)
{
  GaitDynamicPhaseController controller(0.0, staticParameters(), trot(1.0, 0.4));
  controller.update(0.2, trot(2.0, 0.4), 0.1);

  // At the change time phases are from the previous segment (as mode at event time), new phase after it
  EXPECT_NEAR(controller.getPhasesAtTime(0.2)[0], 0.7, 1e-9);
  EXPECT_NEAR(controller.getBasePhaseAfterTime(0.2), 0.1, 1e-9);
  EXPECT_NEAR(controller.getBasePhaseAfterTime(0.1), 0.6, 1e-9);
  EXPECT_NEAR(controller.getPhasesAtTime(0.3)[0], 0.3, 1e-9);
  EXPECT_NEAR(controller.getBasePhaseAfterTime(0.3), 0.3, 1e-9);
  EXPECT_NEAR(controller.getPhasesAtTime(0.3)[1], 0.8, 1e-9);
}

TEST(GaitPlannerRephasingTest, startFromStandingKeepsAllContacts)
{
  GaitPlanner planner(staticParameters(), standing(), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Trot without flight phase (swing ratio below 0.5) can start with all end effectors in contact
  const auto target = trot(1.3, 0.45);
  scalar_t initTime = 0.5;
  const scalar_t changeTime = initTime + UPDATE_OFFSET;
  auto modeSchedule = mpcIteration(planner, initTime, &target);
  EXPECT_EQ(planner.getDynamicParameters(), target);
  EXPECT_EQ(planner.getContactFlagsAtTime(changeTime + 1e-3), ALL_IN_CONTACT);

  // First lift off is soon (end of four end effectors stance), at least two end effectors are in contact
  EXPECT_LT(planner.getContactFlagsAtTime(changeTime + 2.0 * Definitions::REPHASING_MARGIN_TIME).count(), 4);
  for(int k = 0; k < 40; ++k)
  {
    for(const size_t mode: modeSchedule.modeSequence)
    {
      EXPECT_GE(contact_flags_t(mode).count(), 2);
    }
    initTime += MPC_PERIOD;
    modeSchedule = mpcIteration(planner, initTime);
  }
}

TEST(GaitPlannerRephasingTest, startFromStandingFlyingTrot)
{
  GaitPlanner planner(staticParameters(), standing(), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Swing ratio above 0.5: one pair of end effectors has to lift off, without re-phasing (phase 0.5)
  // all of them would be in swing (flight phase) at once
  const auto target = trot(1.33, 0.529);
  scalar_t initTime = 0.5;
  const scalar_t changeTime = initTime + UPDATE_OFFSET;
  mpcIteration(planner, initTime, &target);
  EXPECT_EQ(planner.getDynamicParameters(), target);

  const auto flags = planner.getContactFlagsAtTime(changeTime + 1e-3);
  EXPECT_EQ(flags.count(), 2);
  EXPECT_TRUE(flags == contact_flags_t(9) || flags == contact_flags_t(6));

  // Lifted end effectors are at the start of the swing (they have almost whole swing)
  const auto phases = planner.getPhasesAtTime(changeTime + 1e-3);
  const scalar_t maximumPhase = (target.swingRatio - 0.5) + target.steppingFrequency *
    (Definitions::REPHASING_MARGIN_TIME + 1e-3) + 1e-6;
  for(size_t i = 0; i < 4; ++i)
  {
    if(!flags[i])
    {
      EXPECT_LT(phases[i], maximumPhase) << "end effector " << i;
    }
  }

  // Normal flying trot afterwards
  for(int k = 0; k < 40; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
  }
}

TEST(GaitPlannerRephasingTest, gaitTypeChangeWithTheSameContacts)
{
  GaitPlanner planner(staticParameters(), trot(1.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Offsets equal modulo 1 are not a gait type change, nothing happens to phases
  auto sameTrot = trot(1.0, 0.4);
  sameTrot.phaseOffsets = {0.5, 0.5, 0.0};
  scalar_t initTime = 0.1;
  mpcIteration(planner, initTime, &sameTrot);
  EXPECT_EQ(planner.getDynamicParameters(), sameTrot);
  EXPECT_NEAR(planner.getPhasesAtTime(initTime + UPDATE_OFFSET + 0.1)[0], 0.5 + initTime + UPDATE_OFFSET + 0.1, 1e-9);

  // Small change of offsets with the same contact states: phase is continuous
  auto otherTrot = sameTrot;
  otherTrot.phaseOffsets = {0.5, 0.45, 0.0};
  initTime += MPC_PERIOD;
  const scalar_t changeTime = initTime + UPDATE_OFFSET;
  const auto flagsBefore = planner.getContactFlagsAtTime(changeTime + 1e-3);
  mpcIteration(planner, initTime, &otherTrot);
  EXPECT_EQ(planner.getDynamicParameters(), otherTrot);
  EXPECT_EQ(planner.getContactFlagsAtTime(changeTime + 1e-3), flagsBefore);
  EXPECT_NEAR(planner.getPhasesAtTime(changeTime + 1e-3)[0], normalizePhase(0.5 + changeTime + 1e-3), 1e-9);
}

TEST(GaitPlannerRephasingTest, gaitTypeChangeWhileWalking)
{
  GaitPlanner planner(staticParameters(), trot(1.0, 0.4), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // At change time 0.75 base phase is 0.25: end effectors 0 and 3 in swing, 1 and 2 in contact
  scalar_t initTime = 0.5;
  const scalar_t changeTime = initTime + UPDATE_OFFSET;
  const auto flagsBefore = planner.getContactFlagsAtTime(changeTime + 1e-3);
  EXPECT_EQ(flagsBefore, contact_flags_t(6));

  // Pace needs end effectors 0, 2 and 1, 3 in the same state: at least two contacts change,
  // from such phases the one with the most contacts is chosen
  const auto target = pace(1.0, 0.4);
  mpcIteration(planner, initTime, &target);
  EXPECT_EQ(planner.getDynamicParameters(), target);
  const auto flagsAfter = planner.getContactFlagsAtTime(changeTime + 1e-3);
  EXPECT_EQ(changedContacts(flagsBefore, flagsAfter), 2);
  EXPECT_EQ(flagsAfter, ALL_IN_CONTACT);

  for(int k = 0; k < 40; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
  }

  // Back to trot: pace in four end effectors stance at change time, contacts are kept
  for(int k = 0; k < 40; ++k)
  {
    if(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 1e-3) == ALL_IN_CONTACT) break;
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
  }
  const scalar_t secondChangeTime = initTime + UPDATE_OFFSET;
  ASSERT_EQ(planner.getContactFlagsAtTime(secondChangeTime + 1e-3), ALL_IN_CONTACT);
  const auto backToTrot = trot(1.0, 0.4);
  mpcIteration(planner, initTime, &backToTrot);
  EXPECT_EQ(planner.getContactFlagsAtTime(secondChangeTime + 1e-3), ALL_IN_CONTACT);
}

TEST(GaitPlannerRephasingTest, stopKeepsMostContacts)
{
  // Flying trot: there is no phase with all end effectors in contact
  GaitPlanner planner(staticParameters(), trot(2.0, 0.529), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Find change time in flight phase (planned at the change time)
  scalar_t initTime = 0.0;
  for(int k = 0; k < 400; ++k)
  {
    const scalar_t changeTime = initTime + UPDATE_OFFSET;
    if(planner.getContactFlagsAtTime(changeTime - 2e-3).none() &&
      planner.getContactFlagsAtTime(changeTime + 2e-3).none()) break;
    initTime += 1e-3;
  }
  ASSERT_TRUE(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET).none());
  planner.getModeSchedule(initTime, initTime + HORIZON);

  // Stop with the same gait: robot stands on two end effectors, not in flight
  auto stop = trot(0.0, 0.529);
  auto modeSchedule = mpcIteration(planner, initTime, &stop);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 0.5).count(), 2);
  EXPECT_EQ(contact_flags_t(modeSchedule.modeAtTime(initTime + UPDATE_OFFSET + 0.5)).count(), 2);

  // Standing robot puts down end effectors when the swing ratio allows it
  const auto stopAllContacts = trot(0.0, 0.0);
  initTime += MPC_PERIOD;
  modeSchedule = mpcIteration(planner, initTime, &stopAllContacts);
  EXPECT_EQ(planner.getDynamicParameters(), stopAllContacts);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 0.5), ALL_IN_CONTACT);
  EXPECT_EQ(contact_flags_t(modeSchedule.modeAtTime(initTime + UPDATE_OFFSET + 0.5)), ALL_IN_CONTACT);
}

TEST(GaitPlannerRephasingTest, stopWithGaitTypeChange)
{
  GaitPlanner planner(staticParameters(), trot(1.3, 0.529), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Stop with standing gait of config (all end effectors with the same phase)
  scalar_t initTime = 0.3;
  const auto target = standing();
  mpcIteration(planner, initTime, &target);
  EXPECT_EQ(planner.getDynamicParameters(), target);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 0.5), ALL_IN_CONTACT);
}

TEST(GaitPlannerRephasingTest, standingRobotWaitsForStart)
{
  GaitPlanner planner(staticParameters(), standing(), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  // Flying trot while standing would lift two end effectors, change waits for start
  const auto standingTrot = trot(0.0, 0.529);
  scalar_t initTime = 0.2;
  mpcIteration(planner, initTime, &standingTrot);
  EXPECT_EQ(planner.getDynamicParameters(), standing());
  EXPECT_EQ(planner.getTargetDynamicParameters(), standingTrot);
  for(int k = 0; k < 10; ++k)
  {
    initTime += MPC_PERIOD;
    mpcIteration(planner, initTime);
    EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 0.5), ALL_IN_CONTACT);
  }

  const auto walking = trot(1.33, 0.529);
  initTime += MPC_PERIOD;
  mpcIteration(planner, initTime, &walking);
  EXPECT_EQ(planner.getDynamicParameters(), walking);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET + 1e-3).count(), 2);
}

// Sequence of the simulation test: start, 1.3 Hz -> 2.3 Hz -> 1.3 Hz, stop
TEST(GaitPlannerRephasingTest, simulationSequence)
{
  GaitPlanner planner(staticParameters(), standing(), 0.0);
  planner.getModeSchedule(0.0, HORIZON);

  scalar_t initTime = 0.0;
  const auto run = [&](const GaitDynamicParameters* target, int iterations)
  {
    initTime += MPC_PERIOD;
    auto modeSchedule = mpcIteration(planner, initTime, target);
    for(int k = 0; k < iterations; ++k)
    {
      initTime += MPC_PERIOD;
      modeSchedule = mpcIteration(planner, initTime);
      // All end effectors in swing never longer than flight phase of the slowest trot
      const scalar_t maximumFlight = (0.529 - 0.5) / 1.33 + 1e-3;
      for(size_t i = 1; i + 1 < modeSchedule.modeSequence.size(); ++i)
      {
        if(modeSchedule.modeSequence[i] != 0 || modeSchedule.eventTimes[i] < initTime) continue;
        EXPECT_LT(modeSchedule.eventTimes[i] - modeSchedule.eventTimes[i - 1], maximumFlight)
          << "flight ending at " << modeSchedule.eventTimes[i];
      }
    }
  };

  const auto slow = trot(1.33, 0.529);
  const auto fast = trot(2.33, 0.529);
  const auto stop = standing();

  const scalar_t startTime = initTime + MPC_PERIOD + UPDATE_OFFSET;
  run(&slow, 0);
  EXPECT_EQ(planner.getContactFlagsAtTime(startTime + 1e-3).count(), 2);
  run(nullptr, 160);
  run(&fast, 80);
  EXPECT_EQ(planner.getDynamicParameters(), fast);
  run(&slow, 80);
  EXPECT_EQ(planner.getDynamicParameters(), slow);
  run(&stop, 20);
  EXPECT_EQ(planner.getContactFlagsAtTime(initTime + UPDATE_OFFSET), ALL_IN_CONTACT);
}
