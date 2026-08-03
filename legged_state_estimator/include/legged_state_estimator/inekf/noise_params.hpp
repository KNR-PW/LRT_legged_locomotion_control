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
#ifndef __LEGGED_STATE_ESTIMATOR_NOISEPARAMS___
#define __LEGGED_STATE_ESTIMATOR_NOISEPARAMS___

#include <iostream>

#include <Eigen/Core>


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
  void setGyroscopeNoise(const ocs2::vector3_t& stddev);
  void setGyroscopeNoise(const ocs2::matrix3_t& cov);

  void setAccelerometerNoise(const ocs2::scalar_t stddev);
  void setAccelerometerNoise(const ocs2::vector3_t& stddev);
  void setAccelerometerNoise(const ocs2::matrix3_t& cov);  

  void setGyroscopeBiasNoise(const ocs2::scalar_t stddev);
  void setGyroscopeBiasNoise(const ocs2::vector3_t& stddev);
  void setGyroscopeBiasNoise(const ocs2::matrix3_t& cov);

  void setAccelerometerBiasNoise(const ocs2::scalar_t stddev);
  void setAccelerometerBiasNoise(const ocs2::vector3_t& stddev);
  void setAccelerometerBiasNoise(const ocs2::matrix3_t& cov);  

  void setContactNoise(const ocs2::scalar_t stddev);
  void setContactNoise(const ocs2::vector3_t& stddev);
  void setContactNoise(const ocs2::matrix3_t& cov);

  const ocs2::matrix3_t& getGyroscopeCov() const;
  const ocs2::matrix3_t& getAccelerometerCov() const;
  const ocs2::matrix3_t& getGyroscopeBiasCov() const;
  const ocs2::matrix3_t& getAccelerometerBiasCov() const;
  const ocs2::matrix3_t& getContactCov() const;

  friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ocs2::matrix3_t Qg_;
  ocs2::matrix3_t Qa_;
  ocs2::matrix3_t Qbg_;
  ocs2::matrix3_t Qba_;
  ocs2::matrix3_t Ql_;
  ocs2::matrix3_t Qc_;
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_NOISEPARAMS___