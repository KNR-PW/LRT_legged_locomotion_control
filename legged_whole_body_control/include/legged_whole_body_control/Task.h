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

#include <cassert>
#include <cstddef>
#include <utility>

#include <legged_whole_body_control/Types.hpp>

namespace legged_whole_body_control
{

class Task
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Task() = default;

  Task(ocs2::matrix_t&& a,
       ocs2::vector_t&& b,
       ocs2::matrix_t&& d,
       ocs2::vector_t&& f)
      : a_(std::move(a)),
        d_(std::move(d)),
        b_(std::move(b)),
        f_(std::move(f))
  {
  }

  explicit Task(std::size_t numDecisionVars)
      : a_(ocs2::matrix_t::Zero(0, numDecisionVars)),
        d_(ocs2::matrix_t::Zero(0, numDecisionVars)),
        b_(ocs2::vector_t::Zero(0)),
        f_(ocs2::vector_t::Zero(0))
  {
  }

  // --------------------------------------------------------------------------
  // Addition
  // --------------------------------------------------------------------------

  // lvalue += lvalue
  Task& operator+=(const Task& rhs)
  {
    a_ = concatenateMatrices(std::move(a_), rhs.a_);
    d_ = concatenateMatrices(std::move(d_), rhs.d_);
    b_ = concatenateVectors(std::move(b_), rhs.b_);
    f_ = concatenateVectors(std::move(f_), rhs.f_);

    return *this;
  }

  // lvalue += temporary
  //
  // This is the important overload for:
  //
  //   weightedTask += weight * formulateTask();
  //
  Task& operator+=(Task&& rhs)
  {
    a_ = concatenateMatrices(std::move(a_), std::move(rhs.a_));
    d_ = concatenateMatrices(std::move(d_), std::move(rhs.d_));
    b_ = concatenateVectors(std::move(b_), std::move(rhs.b_));
    f_ = concatenateVectors(std::move(f_), std::move(rhs.f_));

    return *this;
  }

  // a + b
  Task operator+(const Task& rhs) const&
  {
    Task result = *this;
    result += rhs;
    return result;
  }

  // std::move(a) + b
  //
  // Allows the storage owned by a to be reused.
  friend Task operator+(Task&& lhs, const Task& rhs)
  {
    lhs += rhs;
    return std::move(lhs);
  }

  // --------------------------------------------------------------------------
  // Scalar multiplication
  // --------------------------------------------------------------------------

  // scalar * temporary Task
  //
  // This is important for:
  //
  //   weight * formulateTask()
  //
  // The existing Task buffers are scaled in-place. No additional
  // matrix/vector allocations are required here.
  friend Task operator*(ocs2::scalar_t lhs, Task&& rhs)
  {
    rhs.a_ *= lhs;
    rhs.b_ *= lhs;
    rhs.d_ *= lhs;
    rhs.f_ *= lhs;

    return std::move(rhs);
  }

  // scalar * const Task
  //
  // Required when the Task is an lvalue.
  friend Task operator*(ocs2::scalar_t lhs, const Task& rhs)
  {
    return Task{
        lhs * rhs.a_,
        lhs * rhs.b_,
        lhs * rhs.d_,
        lhs * rhs.f_};
  }

  // Task * scalar, lvalue
  friend Task operator*(const Task& lhs, ocs2::scalar_t rhs)
  {
    return rhs * lhs;
  }

  // Task * scalar, temporary
  friend Task operator*(Task&& lhs, ocs2::scalar_t rhs)
  {
    lhs.a_ *= rhs;
    lhs.b_ *= rhs;
    lhs.d_ *= rhs;
    lhs.f_ *= rhs;

    return std::move(lhs);
  }

public:
  ocs2::matrix_t a_;
  ocs2::matrix_t d_;
  ocs2::vector_t b_;
  ocs2::vector_t f_;

private:
  // --------------------------------------------------------------------------
  // Matrix concatenation
  // --------------------------------------------------------------------------

  // First matrix can be moved from, second one must be preserved.
  static ocs2::matrix_t concatenateMatrices(
      ocs2::matrix_t&& m1,
      const ocs2::matrix_t& m2)
  {
    if (m1.cols() == 0)
    {
      return m2;
    }

    if (m2.cols() == 0)
    {
      return std::move(m1);
    }

    assert(m1.cols() == m2.cols());

    const Eigen::Index rows1 = m1.rows();
    const Eigen::Index rows2 = m2.rows();

    ocs2::matrix_t result(rows1 + rows2, m1.cols());

    result.topRows(rows1) = m1;
    result.bottomRows(rows2) = m2;

    return result;
  }

  // Both matrices can be consumed.
  static ocs2::matrix_t concatenateMatrices(
      ocs2::matrix_t&& m1,
      ocs2::matrix_t&& m2)
  {
    if (m1.cols() == 0)
    {
      return std::move(m2);
    }

    if (m2.cols() == 0)
    {
      return std::move(m1);
    }

    assert(m1.cols() == m2.cols());

    const Eigen::Index rows1 = m1.rows();
    const Eigen::Index rows2 = m2.rows();

    ocs2::matrix_t result(rows1 + rows2, m1.cols());

    result.topRows(rows1) = m1;
    result.bottomRows(rows2) = m2;

    return result;
  }

  // --------------------------------------------------------------------------
  // Vector concatenation
  // --------------------------------------------------------------------------

  // First vector can be moved from, second one must be preserved.
  static ocs2::vector_t concatenateVectors(
      ocs2::vector_t&& v1,
      const ocs2::vector_t& v2)
  {
    if (v1.size() == 0)
    {
      return v2;
    }

    if (v2.size() == 0)
    {
      return std::move(v1);
    }

    const Eigen::Index size1 = v1.size();
    const Eigen::Index size2 = v2.size();

    ocs2::vector_t result(size1 + size2);

    result.head(size1) = v1;
    result.tail(size2) = v2;

    return result;
  }

  // Both vectors can be consumed.
  static ocs2::vector_t concatenateVectors(
      ocs2::vector_t&& v1,
      ocs2::vector_t&& v2)
  {
    if (v1.size() == 0)
    {
      return std::move(v2);
    }

    if (v2.size() == 0)
    {
      return std::move(v1);
    }

    const Eigen::Index size1 = v1.size();
    const Eigen::Index size2 = v2.size();

    ocs2::vector_t result(size1 + size2);

    result.head(size1) = v1;
    result.tail(size2) = v2;

    return result;
  }
};

} // namespace legged_whole_body_control
#endif