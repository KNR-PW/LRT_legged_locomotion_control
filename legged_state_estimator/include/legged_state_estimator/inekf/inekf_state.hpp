/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_state.hpp
 *  @author Ross Hartley
 *  @brief  Header file for InEKFState
 *  @date   September 25, 2018
 **/

#ifndef __LEGGED_STATE_ESTIMATOR_INEKF_STATE___
#define __LEGGED_STATE_ESTIMATOR_INEKF_STATE___

#include <iostream>

#include <Eigen/Core>


namespace legged_state_estimator {

enum StateType {WorldCentric, BodyCentric};

class InEKFState {
public:
  InEKFState();
  InEKFState(const ocs2::matrix_t& X);
  InEKFState(const ocs2::matrix_t& X, const ocs2::vector_t& Theta);
  InEKFState(const ocs2::matrix_t& X, const ocs2::vector_t& Theta, const ocs2::matrix_t& P);

  ~InEKFState() = default;

  InEKFState(const InEKFState&) = default;
  InEKFState& operator=(const InEKFState&) = default;
  InEKFState(InEKFState&&) noexcept = default;
  InEKFState& operator=(InEKFState&&) noexcept = default;

  const ocs2::matrix_t& getX() const;
  const ocs2::vector_t& getTheta() const;
  const ocs2::matrix_t& getP() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 3> getRotation() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 1> getVelocity() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 1> getPosition() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 1> getVector(int id) const;
  const Eigen::VectorBlock<const ocs2::vector_t, 3> getGyroscopeBias() const;
  const Eigen::VectorBlock<const ocs2::vector_t, 3> getAccelerometerBias() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 3> getRotationCovariance() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 3> getVelocityCovariance() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 3> getPositionCovariance() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 3> getGyroscopeBiasCovariance() const;
  const Eigen::Block<const ocs2::matrix_t, 3, 3> getAccelerometerBiasCovariance() const;
  int dimX() const;
  int dimTheta() const;
  int dimP() const;
  const StateType getStateType() const;
  const ocs2::matrix_t getWorldX() const;
  const ocs2::matrix3_t getWorldRotation() const;
  const ocs2::vector3_t getWorldVelocity() const;
  const ocs2::vector3_t getWorldPosition() const;
  const ocs2::matrix_t getBodyX() const;
  const ocs2::matrix3_t getBodyRotation() const;
  const ocs2::vector3_t getBodyVelocity() const;
  const ocs2::vector3_t getBodyPosition() const;

  void setX(const ocs2::matrix_t& X);
  void setP(const ocs2::matrix_t& P);
  void setTheta(const ocs2::vector_t& Theta);
  void setRotation(const ocs2::matrix3_t& R);
  void setVelocity(const ocs2::vector3_t& v);
  void setPosition(const ocs2::vector3_t& p);
  void setGyroscopeBias(const ocs2::vector3_t& bg);
  void setAccelerometerBias(const ocs2::vector3_t& ba);
  void setRotationCovariance(const ocs2::matrix3_t& cov);
  void setVelocityCovariance(const ocs2::matrix3_t& cov);
  void setPositionCovariance(const ocs2::matrix3_t& cov);
  void setGyroscopeBiasCovariance(const ocs2::matrix3_t& cov);
  void setAccelerometerBiasCovariance(const ocs2::matrix3_t& cov);
  void copyDiagX(const int n, ocs2::matrix_t& BigX) const;
  void copyDiagXinv(const int n, ocs2::matrix_t& BigXinv) const;

  ocs2::matrix_t calcXinv() const;

  friend std::ostream& operator<<(std::ostream& os, const InEKFState& s);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  StateType state_type_ = StateType::WorldCentric; 
  ocs2::matrix_t X_;
  ocs2::vector_t Theta_;
  ocs2::matrix_t P_;
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_INEKF_STATE___