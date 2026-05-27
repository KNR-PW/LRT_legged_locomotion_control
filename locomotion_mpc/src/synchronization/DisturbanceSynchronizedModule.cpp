
#include <legged_locomotion_mpc/synchronization/DisturbanceSynchronizedModule.hpp>
#include <iostream>

namespace legged_locomotion_mpc 
{
  namespace synchronization
  {

    using namespace ocs2;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    DisturbanceSynchronizedModule::DisturbanceSynchronizedModule():
    bufferedDisturbance_(vector6_t::Zero()) {}

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void DisturbanceSynchronizedModule::preSolverRun(
      ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
      const ocs2::ReferenceManagerInterface& referenceManager)
    {
      bufferedDisturbance_.updateFromBuffer();
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void DisturbanceSynchronizedModule::postSolverRun(const ocs2::PrimalSolution& primalSolution) {}

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void DisturbanceSynchronizedModule::updateDistrubance(ocs2::scalar_t time, 
      const vector6_t& disturbance)
    {
      bufferedDisturbance_.setBuffer(disturbance);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    const vector6_t& DisturbanceSynchronizedModule::getCurrentDisturbance() const
    {
      return bufferedDisturbance_.get();
    }
  } // namespace synchronization
} // namespace legged_locomotion_mpc

