/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/

#include "legged_state_estimator/inekf/noise_params.hpp"


namespace legged_state_estimator {

// ------------ NoiseParams -------------
// Default Constructor
NoiseParams::NoiseParams() {
  setGyroscopeNoise(0.01);
  setAccelerometerNoise(0.1);
  setGyroscopeBiasNoise(0.00001);
  setAccelerometerBiasNoise(0.0001);
  setContactNoise(0.1);
}

void NoiseParams::setGyroscopeNoise(const ocs2::scalar_t stddev) { Qg_ = stddev*stddev*matrix3_t::Identity(); }
void NoiseParams::setGyroscopeNoise(const vector3_t& stddev) { Qg_ << stddev(0)*stddev(0),0,0, 0,stddev(1)*stddev(1),0, 0,0,stddev(2)*stddev(2); }
void NoiseParams::setGyroscopeNoise(const matrix3_t& cov) { Qg_ = cov; }

void NoiseParams::setAccelerometerNoise(const ocs2::scalar_t stddev) { Qa_ = stddev*stddev*matrix3_t::Identity(); }
void NoiseParams::setAccelerometerNoise(const vector3_t& stddev) { Qa_ << stddev(0)*stddev(0),0,0, 0,stddev(1)*stddev(1),0, 0,0,stddev(2)*stddev(2); }
void NoiseParams::setAccelerometerNoise(const matrix3_t& cov) { Qa_ = cov; } 

void NoiseParams::setGyroscopeBiasNoise(const ocs2::scalar_t stddev) { Qbg_ = stddev*stddev*matrix3_t::Identity(); }
void NoiseParams::setGyroscopeBiasNoise(const vector3_t& stddev) { Qbg_ << stddev(0)*stddev(0),0,0, 0,stddev(1)*stddev(1),0, 0,0,stddev(2)*stddev(2); }
void NoiseParams::setGyroscopeBiasNoise(const matrix3_t& cov) { Qbg_ = cov; }

void NoiseParams::setAccelerometerBiasNoise(const ocs2::scalar_t stddev) { Qba_ = stddev*stddev*matrix3_t::Identity(); }
void NoiseParams::setAccelerometerBiasNoise(const vector3_t& stddev) { Qba_ << stddev(0)*stddev(0),0,0, 0,stddev(1)*stddev(1),0, 0,0,stddev(2)*stddev(2); }
void NoiseParams::setAccelerometerBiasNoise(const matrix3_t& cov) { Qba_ = cov; }

void NoiseParams::setContactNoise(const ocs2::scalar_t stddev) { Qc_ = stddev*stddev*matrix3_t::Identity(); }
void NoiseParams::setContactNoise(const vector3_t& stddev) { Qc_ << stddev(0)*stddev(0),0,0, 0,stddev(1)*stddev(1),0, 0,0,stddev(2)*stddev(2); }
void NoiseParams::setContactNoise(const matrix3_t& cov) { Qc_ = cov; }

const matrix3_t& NoiseParams::getGyroscopeCov() const { return Qg_; }
const matrix3_t& NoiseParams::getAccelerometerCov() const { return Qa_; }
const matrix3_t& NoiseParams::getGyroscopeBiasCov() const { return Qbg_; }
const matrix3_t& NoiseParams::getAccelerometerBiasCov() const { return Qba_; }
const matrix3_t& NoiseParams::getContactCov() const { return Qc_; }

std::ostream& operator<<(std::ostream& os, const NoiseParams& p) {
  os << "--------- Noise Params -------------" << std::endl;
  os << "Gyroscope Covariance:\n" << p.Qg_ << std::endl;
  os << "Accelerometer Covariance:\n" << p.Qa_ << std::endl;
  os << "Gyroscope Bias Covariance:\n" << p.Qbg_ << std::endl;
  os << "Accelerometer Bias Covariance:\n" << p.Qba_ << std::endl;
  os << "Contact Covariance:\n" << p.Qc_ << std::endl;
  os << "-----------------------------------" << std::endl;
  return os;
}

} // end inekf namespace
