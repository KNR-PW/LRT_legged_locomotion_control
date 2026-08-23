/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   noise_params.hpp
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/
#ifndef __LEGGED_STATE_ESTIMATOR_NOISEPARAMS__
#define __LEGGED_STATE_ESTIMATOR_NOISEPARAMS__

#include <iostream>

#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

class NoiseParams {
public:
  NoiseParams();

  ~NoiseParams() = default;

  NoiseParams(const NoiseParams&) = default;
  NoiseParams& operator=(const NoiseParams&) = default;
  NoiseParams(NoiseParams&&) noexcept = default;
  NoiseParams& operator=(NoiseParams&&) noexcept = default;

  void setGyroscopeNoise(const ocs2::scalar_t stddev);
  void setGyroscopeNoise(const vector3_t& stddev);
  void setGyroscopeNoise(const matrix3_t& cov);

  void setAccelerometerNoise(const ocs2::scalar_t stddev);
  void setAccelerometerNoise(const vector3_t& stddev);
  void setAccelerometerNoise(const matrix3_t& cov);  

  void setGyroscopeBiasNoise(const ocs2::scalar_t stddev);
  void setGyroscopeBiasNoise(const vector3_t& stddev);
  void setGyroscopeBiasNoise(const matrix3_t& cov);

  void setAccelerometerBiasNoise(const ocs2::scalar_t stddev);
  void setAccelerometerBiasNoise(const vector3_t& stddev);
  void setAccelerometerBiasNoise(const matrix3_t& cov);  

  void setContactNoise(const ocs2::scalar_t stddev);
  void setContactNoise(const vector3_t& stddev);
  void setContactNoise(const matrix3_t& cov);

  const matrix3_t& getGyroscopeCov() const;
  const matrix3_t& getAccelerometerCov() const;
  const matrix3_t& getGyroscopeBiasCov() const;
  const matrix3_t& getAccelerometerBiasCov() const;
  const matrix3_t& getContactCov() const;

  friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  matrix3_t Qg_;
  matrix3_t Qa_;
  matrix3_t Qbg_;
  matrix3_t Qba_;
  matrix3_t Ql_;
  matrix3_t Qc_;
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_NOISEPARAMS__