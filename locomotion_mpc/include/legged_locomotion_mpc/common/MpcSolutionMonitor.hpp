// Copyright (c) 2026, Bartłomiej Krajewski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __MPC_SOLUTION_MONITOR_LEGGED_LOCOMOTION_MPC__
#define __MPC_SOLUTION_MONITOR_LEGGED_LOCOMOTION_MPC__

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <ocs2_core/Types.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>

namespace legged_locomotion_mpc
{
  /**
   * Detects diverged MPC solutions from performance indices of the solver. Every solution starts from
   * the previous one, so a diverged solution makes the next ones worse: it should not be used and solver
   * has to be reset (next solution starts from the initializer). First solutions after the reset can be
   * still not good enough, so a few of them are only rejected before the next reset.
   */
  class MpcSolutionMonitor
  {
    public:

      enum class Result
      {
        VALID,
        DIVERGED,      // Do not use the solution
        DIVERGED_RESET // Do not use the solution, reset the solver
      };

      /** Disabled, every solution is valid */
      MpcSolutionMonitor() = default;

      /**
       * @param [in] maxDynamicsViolation: maximum dynamics violation (sum of squared errors) of valid
       * solution, 0.0: disabled, every solution is valid
       * @param [in] recoverySolutions: number of solutions (with the one that reset the solver),
       * after which the solver is reset again
       */
      explicit MpcSolutionMonitor(ocs2::scalar_t maxDynamicsViolation, size_t recoverySolutions = 5):
        maxDynamicsViolation_(maxDynamicsViolation), recoverySolutions_(recoverySolutions)
      {
        if(!std::isfinite(maxDynamicsViolation) || maxDynamicsViolation < 0.0)
        {
          throw std::invalid_argument("[MpcSolutionMonitor]: Maximum dynamics violation has to be finite and not negative!");
        }
        if(recoverySolutions == 0)
        {
          throw std::invalid_argument("[MpcSolutionMonitor]: Number of recovery solutions has to be positive!");
        }
      }

      /** Checks new solution, has to be called once for every solution */
      Result update(const ocs2::PerformanceIndex& performanceIndex)
      {
        if(maxDynamicsViolation_ == 0.0) return Result::VALID;

        // Solver can not recover from not finite solution
        const bool finite = std::isfinite(performanceIndex.cost) &&
          std::isfinite(performanceIndex.dynamicsViolationSSE) &&
          std::isfinite(performanceIndex.equalityConstraintsSSE);

        if(finite && performanceIndex.dynamicsViolationSSE <= maxDynamicsViolation_)
        {
          divergedSolutions_ = 0;
          return Result::VALID;
        }

        const bool reset = !finite || (divergedSolutions_ % recoverySolutions_) == 0;
        ++divergedSolutions_;
        return reset ? Result::DIVERGED_RESET : Result::DIVERGED;
      }

      /** Number of diverged solutions in a row */
      size_t getDivergedSolutions() const
      {
        return divergedSolutions_;
      }

      void reset()
      {
        divergedSolutions_ = 0;
      }

    private:

      ocs2::scalar_t maxDynamicsViolation_ = 0.0;
      size_t recoverySolutions_ = 5;
      size_t divergedSolutions_ = 0;
  };
} // namespace legged_locomotion_mpc

#endif
