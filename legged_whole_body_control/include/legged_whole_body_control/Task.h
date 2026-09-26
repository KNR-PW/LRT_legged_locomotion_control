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

#ifndef __LEGGED_WHOLE_BODY_CONTROL_TASK__
#define __LEGGED_WHOLE_BODY_CONTROL_TASK__

#include <utility>

#include <legged_whole_body_control/Types.hpp>

namespace legged_whole_body_control 
{
  class Task 
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Task() = default;

      Task(ocs2::matrix_t a, ocs2::vector_t b, ocs2::matrix_t d, ocs2::vector_t f): 
        a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

      explicit Task(size_t numDecisionVars): 
        Task(ocs2::matrix_t::Zero(0, numDecisionVars), ocs2::vector_t::Zero(0), 
        ocs2::matrix_t::Zero(0, numDecisionVars), ocs2::vector_t::Zero(0)) {}

      Task operator+(const Task& rhs) const 
      {
        return {concatenateMatrices(std::move(a_), std::move(rhs.a_)), 
          concatenateVectors(std::move(b_), std::move(rhs.b_)), 
          concatenateMatrices(std::move(d_), std::move(rhs.d_)), 
          concatenateVectors(std::move(f_), std::move(rhs.f_))};
      }

      Task operator*(ocs2::scalar_t rhs) const // clang-format off
      {  
        return {a_.cols() > 0 ? rhs * a_ : a_,
          b_.cols() > 0 ? rhs * b_ : b_,
          d_.cols() > 0 ? rhs * d_ : d_,
          f_.cols() > 0 ? rhs * f_ : f_};  // clang-format on
      }

      ocs2::matrix_t a_, d_;
      ocs2::vector_t b_, f_;

      static ocs2::matrix_t concatenateMatrices(ocs2::matrix_t&& m1, ocs2::matrix_t&& m2) 
      {
        if (m1.cols() <= 0) 
        {
          return m2;
        } 
        else if (m2.cols() <= 0) 
        {
          return m1;
        }
        assert(m1.cols() == m2.cols());
        ocs2::matrix_t res(m1.rows() + m2.rows(), m1.cols());
        res << m1, m2;
        return res;
      }

      static ocs2::matrix_t concatenateMatrices(const ocs2::matrix_t& m1, 
        const ocs2::matrix_t& m2) 
      {
        if (m1.cols() <= 0) 
        {
          return m2;
        } 
        else if (m2.cols() <= 0) 
        {
          return m1;
        }
        assert(m1.cols() == m2.cols());
        ocs2::matrix_t res(m1.rows() + m2.rows(), m1.cols());
        res << m1, m2;
        return res;
      }

      static ocs2::vector_t concatenateVectors(ocs2::vector_t&& v1, ocs2::vector_t&& v2) 
      {
        if (v1.cols() <= 0) 
        {
          return v2;
        } 
        else if (v2.cols() <= 0) 
        {
          return v1;
        }
        assert(v1.cols() == v2.cols());
        ocs2::vector_t res(v1.rows() + v2.rows());
        res << v1, v2;
        return res;
      }

      static ocs2::vector_t concatenateVectors(const ocs2::vector_t& v1, 
        const ocs2::vector_t& v2) 
      {
        if (v1.cols() <= 0) 
        {
          return v2;
        } 
        else if (v2.cols() <= 0) 
        {
          return v1;
        }
        assert(v1.cols() == v2.cols());
        ocs2::vector_t res(v1.rows() + v2.rows());
        res << v1, v2;
        return res;
      }
  };
} // namespace legged_whole_body_control
#endif