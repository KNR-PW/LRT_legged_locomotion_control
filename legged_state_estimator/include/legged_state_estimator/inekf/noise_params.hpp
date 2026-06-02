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
#ifndef LEGGED_STATE_ESTIMATOR_NOISEPARAMS_HPP_
#define LEGGED_STATE_ESTIMATOR_NOISEPARAMS_HPP_

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

  void setGyroscopeNoise(const double stddev);
  void setGyroscopeNoise(const Eigen::Vector3d& stddev);
  void setGyroscopeNoise(const Eigen::Matrix3d& cov);

  void setAccelerometerNoise(const double stddev);
  void setAccelerometerNoise(const Eigen::Vector3d& stddev);
  void setAccelerometerNoise(const Eigen::Matrix3d& cov);  

  void setGyroscopeBiasNoise(const double stddev);
  void setGyroscopeBiasNoise(const Eigen::Vector3d& stddev);
  void setGyroscopeBiasNoise(const Eigen::Matrix3d& cov);

  void setAccelerometerBiasNoise(const double stddev);
  void setAccelerometerBiasNoise(const Eigen::Vector3d& stddev);
  void setAccelerometerBiasNoise(const Eigen::Matrix3d& cov);  

  void setContactNoise(const double stddev);
  void setContactNoise(const Eigen::Vector3d& stddev);
  void setContactNoise(const Eigen::Matrix3d& cov);

  const Eigen::Matrix3d& getGyroscopeCov() const;
  const Eigen::Matrix3d& getAccelerometerCov() const;
  const Eigen::Matrix3d& getGyroscopeBiasCov() const;
  const Eigen::Matrix3d& getAccelerometerBiasCov() const;
  const Eigen::Matrix3d& getContactCov() const;

  friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Eigen::Matrix3d Qg_;
  Eigen::Matrix3d Qa_;
  Eigen::Matrix3d Qbg_;
  Eigen::Matrix3d Qba_;
  Eigen::Matrix3d Ql_;
  Eigen::Matrix3d Qc_;
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_NOISEPARAMS_HPP_