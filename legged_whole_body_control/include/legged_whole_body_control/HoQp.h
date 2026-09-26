//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

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
 * Modified by: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __LEGGED_WHOLE_BODY_CONTROL_HOQP__
#define __LEGGED_WHOLE_BODY_CONTROL_HOQP__

#include <memory>

#include "legged_whole_body_control/Task.h"

namespace legged_whole_body_control 
{
  // Hierarchical Optimization Quadratic Program
  class HoQp 
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      using HoQpPtr = std::shared_ptr<HoQp>;

      explicit HoQp(const Task& task) : HoQp(task, nullptr) {};

      HoQp(Task task, HoQpPtr higherProblem);

      ocs2::matrix_t getStackedZMatrix() const { return stackedZ_; }

      Task getStackedTasks() const { return stackedTasks_; }

      ocs2::vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

      ocs2::vector_t getSolutions() const 
      {
        ocs2::vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
        return x;
      }

      size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

    private:
      void initVars();
      void formulateProblem();
      void solveProblem();
      
      void buildHMatrix();
      void buildCVector();
      void buildDMatrix();
      void buildFVector();
      
      void buildZMatrix();
      void stackSlackSolutions();
      
      Task task_, stackedTasksPrev_, stackedTasks_;
      HoQpPtr higherProblem_;
      
      bool hasEqConstraints_{}, hasIneqConstraints_{};
      size_t numSlackVars_{}, numDecisionVars_{};
      ocs2::matrix_t stackedZPrev_, stackedZ_;
      ocs2::vector_t stackedSlackSolutionsPrev_, xPrev_;
      size_t numPrevSlackVars_{};
      
      matrix_row_major_t h_, d_;
      ocs2::vector_t c_, f_;
      ocs2::vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;
      
      // Convenience matrices that are used multiple times
      ocs2::matrix_t eyeNvNv_;
      ocs2::matrix_t zeroNvNx_;
  };
} // namespace legged_whole_body_control
#endif