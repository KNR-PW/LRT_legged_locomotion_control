/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf.hpp
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#ifndef __LEGGED_STATE_ESTIMATOR_INEKF__
#define __LEGGED_STATE_ESTIMATOR_INEKF__

#include <iostream>
#include <map>
#include <algorithm>

#include "Eigen/LU"
#include "unsupported/Eigen/MatrixFunctions"

#include "legged_state_estimator/types.hpp"

#include "legged_state_estimator/inekf/inekf_state.hpp"
#include "legged_state_estimator/inekf/noise_params.hpp"
#include "legged_state_estimator/inekf/lie_group.hpp"
#include "legged_state_estimator/inekf/observations.hpp"


namespace legged_state_estimator {

enum ErrorType {LeftInvariant, RightInvariant};

class InEKF {
public:
/// @name Constructors
/// @{
  /**
   * Default Constructor. Initializes the filter with default state (identity rotation, zero velocity, zero position) and noise parameters. 
   * No contacts, prior landmarks, or magnetic field is set.            gfc = [

    */
  InEKF();
  /**
   * Initialize filter with noise parameters. Initializes th            gfc = [
the default (identity rotation, zero velocity, zero position).
    * @param params: The noise parameters to be assigned.
    */
  InEKF(const NoiseParams& params);
  /**
   * Initialize filter with state. Initializes the noise par            gfc = [
the default.
    * @param state: The state to be assigned.
    */
  InEKF(const InEKFState& state);
  /**
   * Initialize filter with state and noise parameters.
   * @param state: The state to be assigned.
   * @param params: The noise parameters to be assigned.
   */        
  InEKF(const InEKFState& state, const NoiseParams& params);
  /**
   * Initialize filter with state, noise, and error type.
   * @param state: The state to be assigned.
   * @param params: The noise parameters to be assigned.
   * @param error_type: The type of invariant error to be used (affects covariance).
   */       
  InEKF(const InEKFState& state, const NoiseParams& params, const ErrorType error_type);

  ~InEKF() = default;

  InEKF(const InEKF&) = default;
  InEKF& operator=(const InEKF&) = default;
  InEKF(InEKF&&) noexcept = default;
  InEKF& operator=(InEKF&&) noexcept = default;
/// @}

/// @name Getters
/// @{
  /**
   * Gets the current error type.
   */
  ErrorType getErrorType() const;
  /**
   * Gets the current state estimate.
   */
  const InEKFState& getState() const;
  /**
   * Gets the current noise parameters.
   */
  const NoiseParams& getNoiseParams() const;
  /**
   * Gets the filter's current contact states.
   * @return  map of contact ID and bool that indicates if contact is registed
   */
  const std::map<int, bool>& getContacts() const;
  /**
   * Gets the current estimated contact positions.
   * @return  map of contact ID and associated index in the state matrix X
   */
  const std::map<int, int>& getEstimatedContactPositions() const;

  /**
   * Gets the filter's prior landmarks.
   * @return  map of prior landmark ID and position (as a vector3_t)
   */
  const mapIntVector3d& getPriorLandmarks() const;
  /**
   * Gets the filter's estimated landmarks.
   * @return  map of landmark ID and associated index in the state matrix X
   */
  const std::map<int, int>& getEstimatedLandmarks() const;
  /**
   * Gets the filter's set magnetic field.
   * @return  magnetic field in world frame
   */
  const vector3_t& getMagneticField() const;
/// @}


/// @name Setters
/// @{
  /**
   * Sets the current state estimate
   * @param state: The state estimate to be assigned.
   */
  void setState(const InEKFState& state);
  /**
   * Sets the current noise parameters
   * @param params: The noise parameters to be assigned.
   */
  void setNoiseParams(const NoiseParams& params);
  /**
   * Sets the filter's current contact state.
   * @param contacts: A vector of contact ID and indicator pairs. A true indicator means contact is detected.
   */
  void setContacts(const std::vector<std::pair<int,bool>>& contacts);
  /**
   * Sets the filter's prior landmarks.
   * @param prior_landmarks: A map of prior landmark IDs and associated position in the world frame.
   */
  void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
  /** TODO: Sets magnetic field for untested magnetometer measurement */
  void setMagneticField(const vector3_t& true_magnetic_field);
/// @}


/// @name Basic Utilities
/// @{
  /**
   * Resets the filter
   * Initializes state matrix to identity, removes all augmented states, and assigns default noise parameters.
   */
  void clear();
  /**
   * Removes a single landmark from the filter's prior landmark set.
   * @param landmark_id: The ID for the landmark to remove.
   */
  void RemovePriorLandmarks(const int landmark_id);
  /**
   * Removes a set of landmarks from the filter's prior landmark set.
   * @param landmark_ids: A vector of IDs for the landmarks to remove.
   */
  void RemovePriorLandmarks(const std::vector<int>& landmark_ids);
  /**
   * Removes a single landmark from the filter's estimated landmark set.
   * @param landmark_id: The ID for the landmark to remove.
   */
  void RemoveLandmarks(const int landmark_id);
  /**
   * Removes a set of landmarks from the filter's estimated landmark set.
   * @param landmark_ids: A vector of IDs for the landmarks to remove.
   */
  void RemoveLandmarks(const std::vector<int>& landmark_ids);
  /**
   * Keeps a set of landmarks from the filter's estimated landmark set.
   * @param landmark_ids: A vector of IDs for the landmarks to keep.
   */
  void KeepLandmarks(const std::vector<int>& landmark_ids);
/// @}


/// @name Propagation and Correction Methods
/// @{
  /**
   * Propagates the estimated state mean and covariance forward using inertial measurements. 
   * All landmarks positions are assumed to be static.
   * All contacts velocities are assumed to be zero + Gaussian noise.
   * The propagation model currently assumes that the covariance is for the right invariant error.
   * @param imu_w: IMU angular velocity measurement
   * @param imu_a: IMU linear acceleration measurement
   * @param dt: ocs2::scalar_t indicating how long to integrate the inertial measurements for
   */
  void Propagate(const vector3_t& imu_w, const vector3_t& imu_a, const ocs2::scalar_t dt);
  /**
   * Propagates the estimated state mean and covariance forward using inertial measurements. 
   * All landmarks positions are assumed to be static.
   * All contacts velocities are assumed to be zero + Gaussian noise.
   * The propagation model currently assumes that the covariance is for the right invariant error.
   * @param imu: 6x1 vector containing stacked angular velocity and linear acceleration measurements
   * @param dt: ocs2::scalar_t indicating how long to integrate the inertial measurements for
   */
  void Propagate(const Eigen::Matrix<ocs2::scalar_t,6,1>& imu, const ocs2::scalar_t dt);
  /** 
   * Corrects the state estimate using the measured forward kinematics between the IMU and a set of contact frames.
   * If contact is indicated but not included in the state, the state is augmented to include the estimated contact position.
   * If contact is not indicated but is included in the state, the contact position is marginalized out of the state. 
   * This is a right-invariant measurement model. Example usage can be found in @include kinematics.cpp
   * @param measured_kinematics: the measured kinematics containing the contact id, relative pose measurement in the IMU frame, and covariance
   */
  void CorrectKinematics(const vectorKinematics& measured_kinematics); 
  /** 
   * Corrects the state estimate using the measured position between a set of contact frames and the IMU.
   * If the landmark is not included in the state, the state is augmented to include the estimated landmark position. 
   * This is a right-invariant measurement model.
   * @param measured_landmarks: the measured landmarks containing the contact id, relative position measurement in the IMU frame, and covariance
   */
  void CorrectLandmarks(const vectorLandmarks& measured_landmarks);

  /** TODO: Untested magnetometer measurement*/
  void CorrectMagnetometer(const vector3_t& measured_magnetic_field, const matrix3_t& covariance);
  /** TODO: Untested GPS measurement*/
  void CorrectPosition(const vector3_t& measured_position, const matrix3_t& covariance, const vector3_t& indices);
  /** TODO: Untested contact position measurement*/
  void CorrectContactPosition(const int id, const vector3_t& measured_contact_position, const matrix3_t& covariance, const vector3_t& indices);
/// @} 

/** @example kinematics.cpp
 * Testing
 */

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ErrorType error_type_ = ErrorType::LeftInvariant; 
  bool estimate_bias_ = true;  
  InEKFState state_;
  NoiseParams noise_params_;
  vector3_t g_; // Gravity vector in world frame (z-up)
  std::map<int,bool> contacts_;
  std::map<int,int> estimated_contact_positions_;
  mapIntVector3d prior_landmarks_;
  std::map<int,int> estimated_landmarks_;
  vector3_t magnetic_field_;
  Eigen::LDLT<ocs2::matrix_t> ldlt_;

  ocs2::matrix_t StateTransitionMatrix(const vector3_t& w, const vector3_t& a, ocs2::scalar_t dt);
  ocs2::matrix_t DiscreteNoiseMatrix(const ocs2::matrix_t& Phi, ocs2::scalar_t dt);

  // Corrects state using invariant observation models
  void CorrectRightInvariant(const Observation& obs);
  void CorrectLeftInvariant(const Observation& obs);
  void CorrectRightInvariant(const ocs2::matrix_t& Z, const ocs2::matrix_t& H, const ocs2::matrix_t& N);
  void CorrectLeftInvariant(const ocs2::matrix_t& Z, const ocs2::matrix_t& H, const ocs2::matrix_t& N);
  // void CorrectFullState(const Observation& obs); // TODO
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_INEKF__
