/******************************************************************************
BSD 3-Clause License

Copyright (c) 2019, Ross Hartley
Copyright (c) 2023, mayataka
Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2026
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER__
#define __LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER__

#include <cmath>
#include <stdexcept>

#include <legged_state_estimator/types.hpp>

namespace legged_state_estimator 
{

  ///
  /// @class LowPassFilter
  /// @brief A simple low pass filter.
  ///
  template <typename SCALAR_T, int dim = Eigen::Dynamic>
  class LowPassFilter 
  {
    public:
      using Vector = Eigen::Matrix<SCALAR_T, dim, 1>;

      ///
      /// @brief Constructs a low pass filter.
      /// @param[in] sampling_time Sampling time.
      /// @param[in] cutoff_frequency The cut-off frequency.
      /// @param[in] dynamic_size Size parameter used when the template paramter dim
      /// is set to Eigen::Dynamic.
      ///
      LowPassFilter(const SCALAR_T sampling_time, const SCALAR_T cutoff_frequency,
          const int dynamic_size = 0): estimate_(), alpha_(SCALAR_T(0.0)) 
      {
        if(sampling_time <= 0) 
        {
          throw std::invalid_argument(
            "[LowPassFilter] invalid argment: sampling_time must be positive");
        }

        if(cutoff_frequency <= 0) 
        {
          throw std::invalid_argument(
            "[LowPassFilter] invalid argment: cutoff_frequency must be positive");
        }

        if(dim == Eigen::Dynamic && dynamic_size <= 0) 
        {
          throw std::invalid_argument(
            "[LowPassFilter] invalid argment: dynamic_size must be positive");
        }
        
        const SCALAR_T tau = SCALAR_T(1.0) / (SCALAR_T(2.0) * M_PI * cutoff_frequency);
        alpha_ = tau / (tau + sampling_time);
        if(dim == Eigen::Dynamic) 
        {
          estimate_.resize(dynamic_size);
        }
        estimate_.setZero();
      }

      ///
      /// @brief Default constructor. 
      ///
      LowPassFilter(): estimate_(), alpha_(SCALAR_T(0.0)) { }

      ///
      /// @brief Default destructor. 
      ///
      ~LowPassFilter() = default;

      LowPassFilter(const LowPassFilter&) = default;
      LowPassFilter& operator=(const LowPassFilter&) = default;
      LowPassFilter(LowPassFilter&&) noexcept = default;
      LowPassFilter& operator=(LowPassFilter&&) noexcept = default;

      ///
      /// @brief Reset the filter and reset the estimate to zero. 
      ///
      void reset() 
      {
        estimate_.setZero();
      }

      ///
      /// @brief Reset the filter with input estimate. 
      /// @param[in] estimate An initial estimate. 
      ///
      void reset(const Vector& estimate) 
      {
        estimate_ = estimate;
      }

      ///
      /// @brief Updates the estimate.
      /// @param[in] observation Observation. 
      ///
      void update(const Vector& observation) 
      {
        estimate_.array() *= alpha_;
        estimate_.noalias() += (SCALAR_T(1.0) - alpha_) * observation;
      }

      ///
      /// @brief Gets the estimate.
      /// @return const reference to the estimate.
      ///
      const Vector& getEstimate() const 
      {
        return estimate_;
      }

    private:
      Vector estimate_;
      SCALAR_T alpha_;
  };
} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER__ 