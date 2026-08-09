/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#include "legged_state_estimator/inekf/inekf.hpp"


namespace legged_state_estimator {

using namespace std;

void removeRowAndColumn(ocs2::matrix_t& M, int index);

// Default constructor
InEKF::InEKF() 
  : g_((ocs2::vector_t(3) << 0,0,-9.81).finished()), 
    magnetic_field_((ocs2::vector_t(3) << 0,0,0).finished()) {}

// Constructor with noise params
InEKF::InEKF(const NoiseParams& params) 
  : g_((ocs2::vector_t(3) << 0,0,-9.81).finished()), 
    magnetic_field_((ocs2::vector_t(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    noise_params_(params) {}

// Constructor with initial state
InEKF::InEKF(const InEKFState& state) 
  : g_((ocs2::vector_t(3) << 0,0,-9.81).finished()), 
    magnetic_field_((ocs2::vector_t(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    state_(state) {}

// Constructor with initial state and noise params
InEKF::InEKF(const InEKFState& state, const NoiseParams& params) 
  : g_((ocs2::vector_t(3) << 0,0,-9.81).finished()), 
    magnetic_field_((ocs2::vector_t(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    state_(state), 
    noise_params_(params) {}

// Constructor with initial state, noise params, and error type
InEKF::InEKF(const InEKFState& state, const NoiseParams& params, const ErrorType error_type) 
  : g_((ocs2::vector_t(3) << 0,0,-9.81).finished()), 
    magnetic_field_((ocs2::vector_t(3) << std::cos(1.2049),0,std::sin(1.2049)).finished()), 
    state_(state), 
    noise_params_(params), 
    error_type_(error_type) {}

// Clear all data in the filter
void InEKF::clear() {
  state_ = InEKFState();
  noise_params_ = NoiseParams();
  prior_landmarks_.clear();
  estimated_landmarks_.clear();
  contacts_.clear();
  estimated_contact_positions_.clear();
}

// Returns the robot's current error type
ErrorType InEKF::getErrorType() const { return error_type_; }

// Return robot's current state
const InEKFState& InEKF::getState() const { return state_; }

// Sets the robot's current state
void InEKF::setState(const InEKFState& state) { state_ = state; }

// Return noise params
const NoiseParams& InEKF::getNoiseParams() const { return noise_params_; }

// Sets the filter's noise parameters
void InEKF::setNoiseParams(const NoiseParams& params) { noise_params_ = params; }

// Return filter's prior (static) landmarks
const mapIntVector3d& InEKF::getPriorLandmarks() const { return prior_landmarks_; }

// Set the filter's prior (static) landmarks
void InEKF::setPriorLandmarks(const mapIntVector3d& prior_landmarks) { prior_landmarks_ = prior_landmarks; }

// Return filter's estimated landmarks
const std::map<int,int>& InEKF::getEstimatedLandmarks() const { return estimated_landmarks_; }

// Return filter's estimated landmarks
const std::map<int,int>& InEKF::getEstimatedContactPositions() const { return estimated_contact_positions_; }

// Set the filter's contact state
void InEKF::setContacts(const std::vector<std::pair<int,bool>>& contacts) {
  // Insert new measured contact states
  for(const auto& e : contacts) {
    std::pair<map<int,bool>::iterator,bool> ret = contacts_.insert(e);
    // If contact is already in the map, replace with new value
    if(ret.second==false) {
      ret.first->second = e.second;
    }
  }
  return;
}

// Return the filter's contact state
const std::map<int,bool>& InEKF::getContacts() const { return contacts_; }

// Set the true magnetic field
void InEKF::setMagneticField(const vector3_t& true_magnetic_field) { magnetic_field_ = true_magnetic_field; }

// Get the true magnetic field
const vector3_t& InEKF::getMagneticField() const { return magnetic_field_; }

// Compute Analytical state transition matrix
ocs2::matrix_t InEKF::StateTransitionMatrix(const vector3_t& w, 
                                             const vector3_t& a, 
                                             const ocs2::scalar_t dt) {
  const vector3_t phi = w*dt;
  const matrix3_t G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
  const matrix3_t G1 = Gamma_SO3(phi,1); // TODO: These are also needed for the mean propagation, we should not compute twice
  const matrix3_t G2 = Gamma_SO3(phi,2);
  const matrix3_t G0t = G0.transpose();
  const matrix3_t G1t = G1.transpose();
  const matrix3_t G2t = G2.transpose();
  const matrix3_t G3t = Gamma_SO3(-phi,3);

  // Compute the complicated bias terms (derived for the left invariant case)
  const matrix3_t ax = skew(a);
  const matrix3_t wx = skew(w);
  const matrix3_t wx2 = wx*wx;
  const ocs2::scalar_t dt2 = dt*dt;
  const ocs2::scalar_t dt3 = dt2*dt;
  const ocs2::scalar_t theta = w.norm();
  const ocs2::scalar_t theta2 = theta*theta;
  const ocs2::scalar_t theta3 = theta2*theta;
  const ocs2::scalar_t theta4 = theta3*theta;
  const ocs2::scalar_t theta5 = theta4*theta;
  const ocs2::scalar_t theta6 = theta5*theta;
  const ocs2::scalar_t theta7 = theta6*theta;
  const ocs2::scalar_t thetadt = theta*dt;
  const ocs2::scalar_t thetadt2 = thetadt*thetadt;
  const ocs2::scalar_t thetadt3 = thetadt2*thetadt;
  const ocs2::scalar_t sinthetadt = std::sin(thetadt);
  const ocs2::scalar_t costhetadt = std::cos(thetadt);
  const ocs2::scalar_t sin2thetadt = std::sin(2*thetadt);
  const ocs2::scalar_t cos2thetadt = std::cos(2*thetadt);
  const ocs2::scalar_t thetadtcosthetadt = thetadt*costhetadt;
  const ocs2::scalar_t thetadtsinthetadt = thetadt*sinthetadt;

  matrix3_t Phi25L = G0t*(ax*G2t*dt2 
      + ((sinthetadt-thetadtcosthetadt)/(theta3))*(wx*ax)
      - ((cos2thetadt-4*costhetadt+3)/(4*theta4))*(wx*ax*wx)
      + ((4*sinthetadt+sin2thetadt-4*thetadtcosthetadt-2*thetadt)/(4*theta5))*(wx*ax*wx2)
      + ((thetadt2-2*thetadtsinthetadt-2*costhetadt+2)/(2*theta4))*(wx2*ax)
      - ((6*thetadt-8*sinthetadt+sin2thetadt)/(4*theta5))*(wx2*ax*wx)
      + ((2*thetadt2-4*thetadtsinthetadt-cos2thetadt+1)/(4*theta6))*(wx2*ax*wx2) );

  matrix3_t Phi35L = G0t*(ax*G3t*dt3
      - ((thetadtsinthetadt+2*costhetadt-2)/(theta4))*(wx*ax)
      - ((6*thetadt-8*sinthetadt+sin2thetadt)/(8*theta5))*(wx*ax*wx)
      - ((2*thetadt2+8*thetadtsinthetadt+16*costhetadt+cos2thetadt-17)/(8*theta6))*(wx*ax*wx2)
      + ((thetadt3+6*thetadt-12*sinthetadt+6*thetadtcosthetadt)/(6*theta5))*(wx2*ax)
      - ((6*thetadt2+16*costhetadt-cos2thetadt-15)/(8*theta6))*(wx2*ax*wx)
      + ((4*thetadt3+6*thetadt-24*sinthetadt-3*sin2thetadt+24*thetadtcosthetadt)/(24*theta7))*(wx2*ax*wx2) );

  // TODO: Get better approximation using taylor series when theta < tol
  const ocs2::scalar_t tol =  1e-6;
  if(theta < tol) {
    Phi25L = (1.0/2.0)*ax*dt2;
    Phi35L = (1.0/6.0)*ax*dt3;
  }

  // Fill out analytical state transition matrices
  const int dimX = state_.dimX();
  const int dimTheta = state_.dimTheta();
  const int dimP = state_.dimP();
  ocs2::matrix_t Phi = ocs2::matrix_t::Identity(dimP, dimP);
  if  ((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::LeftInvariant) || 
        (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::RightInvariant)) {
    // Compute left-invariant state transisition matrix
    Phi.template block<3,3>(0,0) = G0t; // Phi_11
    Phi.template block<3,3>(3,0).noalias() = -G0t * skew(G1*a) * dt; // Phi_21
    Phi.template block<3,3>(6,0).noalias() = -G0t * skew(G2*a) * dt2; // Phi_31
    Phi.template block<3,3>(3,3) = G0t; // Phi_22
    Phi.template block<3,3>(6,3) = G0t*dt; // Phi_32
    Phi.template block<3,3>(6,6) = G0t; // Phi_33
    for(int i=5;  i < dimX; ++i) {
      Phi.template block<3,3>((i-2)*3,(i-2)*3) = G0t; // Phi_(3+i)(3+i)
    }
    Phi.template block<3,3>(0,dimP-dimTheta) = -G1t * dt; // Phi_15
    Phi.template block<3,3>(3,dimP-dimTheta) = Phi25L; // Phi_25
    Phi.template block<3,3>(6,dimP-dimTheta) = Phi35L; // Phi_35
    Phi.template block<3,3>(3,dimP-dimTheta+3) = -G1t * dt; // Phi_26
    Phi.template block<3,3>(6,dimP-dimTheta+3).noalias() = -G0t * G2 * dt2; // Phi_36
  } 
  else {
    // Compute right-invariant state transition matrix (Assumes unpropagated state)
    const matrix3_t gx = skew(g_);
    const auto& R = state_.getRotation();
    const auto& v = state_.getVelocity();
    const auto& p = state_.getPosition();
    const matrix3_t RG0 = R*G0;
    const matrix3_t RG1dt = R*G1*dt;
    const matrix3_t RG2dt2 = R*G2*dt2;
    Phi.template block<3,3>(3,0) = gx*dt; // Phi_21
    Phi.template block<3,3>(6,0) = 0.5*gx*dt2; // Phi_31
    Phi.template block<3,3>(6,3) = matrix3_t::Identity()*dt; // Phi_32
    Phi.template block<3,3>(0,dimP-dimTheta) = -RG1dt; // Phi_15
    Phi.template block<3,3>(3,dimP-dimTheta).noalias() = -skew(v+RG1dt*a+g_*dt)*RG1dt + RG0*Phi25L; // Phi_25
    Phi.template block<3,3>(6,dimP-dimTheta).noalias() = -skew(p+v*dt+RG2dt2*a+0.5*g_*dt2)*RG1dt + RG0*Phi35L; // Phi_35
    for(int i=5;  i < dimX; ++i) {
      Phi.template block<3,3>((i-2)*3,dimP-dimTheta).noalias() = -skew(state_.getVector(i))*RG1dt; // Phi_(3+i)5
    }
    Phi.template block<3,3>(3,dimP-dimTheta+3) = -RG1dt; // Phi_26
    Phi.template block<3,3>(6,dimP-dimTheta+3) = -RG2dt2; // Phi_36
  }
  return Phi;
}


// Compute Discrete noise matrix
ocs2::matrix_t InEKF::DiscreteNoiseMatrix(const ocs2::matrix_t& Phi, 
                                           const ocs2::scalar_t dt){
  const int dimX = state_.dimX();
  const int dimTheta = state_.dimTheta();
  const int dimP = state_.dimP();    
  ocs2::matrix_t G = ocs2::matrix_t::Identity(dimP,dimP);
  // Compute G using Adjoint of Xk if needed, otherwise identity (Assumes unpropagated state)
  if((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant) || 
      (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
    G.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.getWorldX()); 
  }

  // Continuous noise covariance 
  ocs2::matrix_t Qc = ocs2::matrix_t::Zero(dimP,dimP); // Landmark noise terms will remain zero
  Qc.template block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
  Qc.template block<3,3>(3,3) = noise_params_.getAccelerometerCov();
  for(auto& e : estimated_contact_positions_) {
    Qc.template block<3,3>(3+3*(e.second-3),3+3*(e.second-3)) = noise_params_.getContactCov(); // Contact noise terms
  }
  // TODO: Use kinematic orientation to map noise from contact frame to body frame (not needed if noise is isotropic)
  Qc.template block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
  Qc.template block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();

  // Noise Covariance Discretization
  const ocs2::matrix_t PhiG = Phi * G;
  ocs2::matrix_t Qd = PhiG * Qc * PhiG.transpose() * dt; // Approximated discretized noise matrix (TODO: compute analytical)
  return Qd;
}


// InEKF Propagation - Inertial Data
void InEKF::Propagate(const vector3_t& imu_w, const vector3_t& imu_a, ocs2::scalar_t dt) {
  // Bias corrected IMU measurements
  const vector3_t w = imu_w - state_.getGyroscopeBias();    // Angular Velocity
  const vector3_t a = imu_a - state_.getAccelerometerBias(); // Linear Acceleration

  // Get current state estimate and dimensions
  const auto& X = state_.getX();
  const ocs2::matrix_t Xinv = state_.calcXinv();
  const auto& P = state_.getP();
  int dimX = state_.dimX();
  int dimP = state_.dimP();
  int dimTheta = state_.dimTheta();

  //  ------------ Propagate Covariance --------------- //
  const ocs2::matrix_t Phi = this->StateTransitionMatrix(w,a,dt);
  const ocs2::matrix_t Qd = this->DiscreteNoiseMatrix(Phi, dt);
  ocs2::matrix_t P_pred = Phi * P * Phi.transpose() + Qd;

  // If we don't want to estimate bias, remove correlation
  if(!estimate_bias_) {
    P_pred.block(0,dimP-dimTheta,dimP-dimTheta,dimTheta).setZero();
    P_pred.block(dimP-dimTheta,0,dimTheta,dimP-dimTheta).setZero();
    P_pred.block(dimP-dimTheta,dimP-dimTheta,dimTheta,dimTheta).setIdentity();
  }    

  //  ------------ Propagate Mean --------------- // 
  const auto& R = state_.getRotation();
  const auto& v = state_.getVelocity();
  const auto& p = state_.getPosition();
  const vector3_t phi = w*dt;
  const matrix3_t G0 = Gamma_SO3(phi,0); // Computation can be sped up by computing G0,G1,G2 all at once
  const matrix3_t G1 = Gamma_SO3(phi,1);
  const matrix3_t G2 = Gamma_SO3(phi,2);

  ocs2::matrix_t X_pred = X;
  if(state_.getStateType() == StateType::WorldCentric) {
    // Propagate world-centric state estimate
    X_pred.template block<3,3>(0,0).noalias() = R * G0;
    X_pred.template block<3,1>(0,3).noalias() = v + (R*G1*a + g_)*dt;
    X_pred.template block<3,1>(0,4).noalias() = p + v*dt + (R*G2*a + 0.5*g_)*dt*dt;
  } else {
    // Propagate body-centric state estimate
    const auto& G0t = G0.transpose();
    X_pred.template block<3,3>(0,0).noalias() = G0t * R;
    X_pred.template block<3,1>(0,3).noalias() = G0t * (v - (G1*a + R*g_)*dt);
    X_pred.template block<3,1>(0,4).noalias() = G0t * (p + v*dt - (G2*a + 0.5*R*g_)*dt*dt);
    for(int i=5;  i < dimX; ++i) {
      X_pred.template block<3,1>(0,i).noalias() = G0t * X.block<3,1>(0,i);
    }
  } 
  //  ------------ Update State --------------- // 
  state_.setX(X_pred);
  state_.setP(P_pred);      
}


void InEKF::Propagate(const Eigen::Matrix<ocs2::scalar_t,6,1>& imu, ocs2::scalar_t dt) {
  Propagate(imu.template head<3>(), imu.template tail<3>(), dt);
}


// Correct State: Right-Invariant Observation
void InEKF::CorrectRightInvariant(const ocs2::matrix_t& Z, 
                                  const ocs2::matrix_t& H, 
                                  const ocs2::matrix_t& N) {
  // Get current state estimate
  const auto& X = state_.getX();
  ocs2::vector_t Theta = state_.getTheta();
  ocs2::matrix_t P = state_.getP();
  const int dimX = state_.dimX();
  const int dimTheta = state_.dimTheta();
  const int dimP = state_.dimP();

  // Remove bias
  Theta = Eigen::Matrix<ocs2::scalar_t,6,1>::Zero();
  P.block<6,6>(dimP-dimTheta,dimP-dimTheta) = 0.0001*Eigen::Matrix<ocs2::scalar_t,6,6>::Identity();
  P.block(0,dimP-dimTheta,dimP-dimTheta,dimTheta).setZero();
  P.block(dimP-dimTheta,0,dimTheta,dimP-dimTheta).setZero();
  // std::cout << "P:\n" << P << std::endl;
  // std::cout << state_ << std::endl;

  // Map from left invariant to right invariant error temporarily
  if(error_type_==ErrorType::LeftInvariant) {
    ocs2::matrix_t Adj = ocs2::matrix_t::Identity(dimP,dimP);
    Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); 
    P.noalias() = (Adj * P * Adj.transpose()).eval(); 
  }

  // Compute Kalman Gain
  const ocs2::matrix_t PHT = P * H.transpose();
  const ocs2::matrix_t S = H * PHT + N;
  ocs2::matrix_t Sinv;
  if(S.rows() <= 3) {
    Sinv = S.inverse();
  }
  else {
    ldlt_.compute(S);
    const int dimS = S.rows();
    Sinv = ldlt_.solve(ocs2::matrix_t::Identity(dimS, dimS));
  }
  const ocs2::matrix_t K = PHT * Sinv;

  // Compute state correction vector
  const ocs2::vector_t delta = K*Z;
  const ocs2::matrix_t dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
  const ocs2::vector_t dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

  // Update state
  const ocs2::matrix_t X_new = dX*X; // Right-Invariant Update
  const ocs2::vector_t Theta_new = Theta + dTheta;

  // Set new state  
  state_.setX(X_new); 
  state_.setTheta(Theta_new);

  // Update Covariance
  const ocs2::matrix_t IKH = ocs2::matrix_t::Identity(dimP,dimP) - K*H;
  ocs2::matrix_t P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

  // Map from right invariant back to left invariant error
  if(error_type_==ErrorType::LeftInvariant) {
    ocs2::matrix_t AdjInv = ocs2::matrix_t::Identity(dimP,dimP);
    AdjInv.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.calcXinv()); 
    P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
  }
  // Set new covariance
  state_.setP(P_new); 
}   


// Correct State: Left-Invariant Observation
void InEKF::CorrectLeftInvariant(const ocs2::matrix_t& Z, 
                                 const ocs2::matrix_t& H, 
                                 const ocs2::matrix_t& N) {
  // Get current state estimate
  const auto& X = state_.getX();
  const auto& Theta = state_.getTheta();
  ocs2::matrix_t P = state_.getP();
  int dimX = state_.dimX();
  int dimTheta = state_.dimTheta();
  int dimP = state_.dimP();

  // Map from right invariant to left invariant error temporarily
  if(error_type_==ErrorType::RightInvariant) {
    ocs2::matrix_t AdjInv = ocs2::matrix_t::Identity(dimP,dimP);
    AdjInv.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(state_.calcXinv()); 
    P = (AdjInv * P * AdjInv.transpose()).eval();
  }

  // Compute Kalman Gain
  const ocs2::matrix_t PHT = P * H.transpose();
  const ocs2::matrix_t S = H * PHT + N;
  ocs2::matrix_t Sinv;
  if(S.rows() <= 3) {
    Sinv = S.inverse();
  }
  else {
    ldlt_.compute(S);
    const int dimS = S.rows();
    Sinv = ldlt_.solve(ocs2::matrix_t::Identity(dimS, dimS));
  }
  const ocs2::matrix_t K = PHT * Sinv;

  // Compute state correction vector
  const ocs2::vector_t delta = K*Z;
  const ocs2::matrix_t dX = Exp_SEK3(delta.segment(0,delta.rows()-dimTheta));
  const ocs2::vector_t dTheta = delta.segment(delta.rows()-dimTheta, dimTheta);

  // Update state
  const ocs2::matrix_t X_new = X*dX; // Left-Invariant Update
  const ocs2::vector_t Theta_new = Theta + dTheta;

  // Set new state
  state_.setX(X_new); 
  state_.setTheta(Theta_new);

  // Update Covariance
  const ocs2::matrix_t IKH = ocs2::matrix_t::Identity(dimP,dimP) - K*H;
  ocs2::matrix_t P_new = IKH * P * IKH.transpose() + K*N*K.transpose(); // Joseph update form

  // Map from left invariant back to right invariant error
  if(error_type_==ErrorType::RightInvariant) {
    ocs2::matrix_t Adj = ocs2::matrix_t::Identity(dimP,dimP);
    Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X_new); 
    P_new = (Adj * P_new * Adj.transpose()).eval(); 
  }

  // Set new covariance
  state_.setP(P_new); 
}   

// Correct state using kinematics measured between imu and contact point
void InEKF::CorrectKinematics(const vectorKinematics& measured_kinematics) {
  ocs2::vector_t Z, Y, b;
  ocs2::matrix_t H, N, PI;

  vector<pair<int,int> > remove_contacts;
  vectorKinematics new_contacts;
  vector<int> used_contact_ids;

  for(vectorKinematicsIterator it=measured_kinematics.begin(); it!=measured_kinematics.end(); ++it) {
    // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
    if(find(used_contact_ids.begin(), used_contact_ids.end(), it->id) != used_contact_ids.end()) { 
      cout << "Duplicate contact ID detected! Skipping measurement.\n";
      continue; 
    } 
    else { 
      used_contact_ids.push_back(it->id); 
    }

    // Find contact indicator for the kinematics measurement
    map<int,bool>::iterator it_contact = contacts_.find(it->id);
    if(it_contact == contacts_.end()) { continue; } // Skip if contact state is unknown
    bool contact_indicated = it_contact->second;

    // See if we can find id estimated_contact_positions
    map<int,int>::iterator it_estimated = estimated_contact_positions_.find(it->id);
    bool found = it_estimated!=estimated_contact_positions_.end();

    if(!contact_indicated && found) {
      // If contact is not indicated and id is found in estimated_contacts_, then remove state
      remove_contacts.push_back(*it_estimated); // Add id to remove list
    } 
    else if(contact_indicated && !found) {
      // If contact is indicated and id is not found i n estimated_contacts_, then augment state
      new_contacts.push_back(*it); // Add to augment list

    } 
    else if(contact_indicated && found) {
      // If contact is indicated and id is found in estimated_contacts_, then correct using kinematics
      const int dimX = state_.dimX();
      const int dimTheta = state_.dimTheta();
      const int dimP = state_.dimP();
      int startIndex;
      // Fill out H
      startIndex = H.rows();
      H.conservativeResize(startIndex+3, dimP);
      H.block(startIndex,0,3,dimP).setZero();
      if(state_.getStateType() == StateType::WorldCentric) {
        H.template block<3,3>(startIndex,6) = -matrix3_t::Identity(); // -I
        H.template block<3,3>(startIndex,3*it_estimated->second-dimTheta) = matrix3_t::Identity(); // I
      } 
      else {
        H.template block<3,3>(startIndex,6) = matrix3_t::Identity(); // I
        H.template block<3,3>(startIndex,3*it_estimated->second-dimTheta) = -matrix3_t::Identity(); // -I
      }
      // Fill out N
      startIndex = N.rows();
      N.conservativeResize(startIndex+3, startIndex+3);
      N.block(startIndex,0,3,startIndex).setZero();
      N.block(0,startIndex,startIndex,3).setZero();
      N.template block<3,3>(startIndex,startIndex).noalias() = state_.getWorldRotation() * it->covariance.block<3,3>(3,3) 
                                                                                          * state_.getWorldRotation().transpose();
      // Fill out Z
      startIndex = Z.rows();
      Z.conservativeResize(startIndex+3, Eigen::NoChange);
      const auto& R = state_.getRotation();
      const auto& p = state_.getPosition();
      const auto& d = state_.getVector(it_estimated->second);  
      if(state_.getStateType() == StateType::WorldCentric) {
        Z.template segment<3>(startIndex).noalias() = R * it->pose.block<3,1>(0,3) - (d - p); 
      } 
      else {
        Z.template segment<3>(startIndex).noalias() = R.transpose() * (it->pose.block<3,1>(0,3) - (p - d)); 
      }
    } 
    else {
      // If contact is not indicated and id is found in estimated_contacts_, then skip
      continue;
    }
  }

  // Correct state using stacked observation
  if(Z.rows()>0) {
    if(state_.getStateType() == StateType::WorldCentric) {
      this->CorrectRightInvariant(Z,H,N);
      // this->CorrectRightInvariant(obs);
    } 
    else {
      // this->CorrectLeftInvariant(obs);
      this->CorrectLeftInvariant(Z,H,N);
    }
  }

  // Remove contacts from state
  if(remove_contacts.size() > 0) {
    ocs2::matrix_t X_rem = state_.getX(); 
    ocs2::matrix_t P_rem = state_.getP();
    for(vector<pair<int,int> >::iterator it=remove_contacts.begin(); it!=remove_contacts.end(); ++it) {
      // Remove row and column from X
      removeRowAndColumn(X_rem, it->second);
      // Remove 3 rows and columns from P
      int startIndex = 3 + 3*(it->second-3);
      removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
      removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
      removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
      // Update all indices for estimated_landmarks and estimated_contact_positions
      for(map<int,int>::iterator it2=estimated_landmarks_.begin(); it2!=estimated_landmarks_.end(); ++it2) {
        if(it2->second > it->second) it2->second -= 1;
      }
      for(map<int,int>::iterator it2=estimated_contact_positions_.begin(); it2!=estimated_contact_positions_.end(); ++it2) {
        if(it2->second > it->second) it2->second -= 1;
      }
      // We also need to update the indices of remove_contacts in the case where multiple contacts are being removed at once
      for(vector<pair<int,int> >::iterator it2=it; it2!=remove_contacts.end(); ++it2) {
        if(it2->second > it->second) it2->second -= 1;
      }
      // Remove from list of estimated contact positions 
      estimated_contact_positions_.erase(it->first);
    }
    // Update state and covariance
    state_.setX(X_rem);
    state_.setP(P_rem);
  }


  // Augment state with newly detected contacts
  if(new_contacts.size() > 0) {
    ocs2::matrix_t X_aug = state_.getX(); 
    ocs2::matrix_t P_aug = state_.getP();
    for(vectorKinematicsIterator it=new_contacts.begin(); it!=new_contacts.end(); ++it) {
      // Initialize new landmark mean
      int startIndex = X_aug.rows();
      X_aug.conservativeResize(startIndex+1, startIndex+1);
      X_aug.block(startIndex,0,1,startIndex).setZero();
      X_aug.block(0,startIndex,startIndex,1).setZero();
      X_aug(startIndex, startIndex) = 1;
      if(state_.getStateType() == StateType::WorldCentric) {
        X_aug.block(0,startIndex,3,1).noalias() = state_.getPosition() + state_.getRotation() * it->pose.block<3,1>(0,3);
      } 
      else {
        X_aug.block(0,startIndex,3,1).noalias() = state_.getPosition() - it->pose.block<3,1>(0,3);
      }

      // Initialize new landmark covariance - TODO:speed up
      ocs2::matrix_t F = ocs2::matrix_t::Zero(state_.dimP()+3,state_.dimP()); 
      F.block(0,0,state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()).setIdentity(); // for old X
      F.block(state_.dimP()-state_.dimTheta()+3,state_.dimP()-state_.dimTheta(),state_.dimTheta(),state_.dimTheta()).setIdentity(); // for theta
      ocs2::matrix_t G = ocs2::matrix_t::Zero(F.rows(),3);
      // Blocks for new contact
      if((state_.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant) || 
          (state_.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
        F.block(state_.dimP()-state_.dimTheta(),6,3,3) = matrix3_t::Identity(); 
        G.block(G.rows()-state_.dimTheta()-3,0,3,3) = state_.getWorldRotation();
      } 
      else {
        F.block(state_.dimP()-state_.dimTheta(),6,3,3) = matrix3_t::Identity(); 
        F.block(state_.dimP()-state_.dimTheta(),0,3,3) = skew(-it->pose.block<3,1>(0,3)); 
        G.block(G.rows()-state_.dimTheta()-3,0,3,3) = matrix3_t::Identity();
      }
      P_aug = (F*P_aug*F.transpose() + G*it->covariance.block<3,3>(3,3)*G.transpose()).eval(); 

      // Update state and covariance
      state_.setX(X_aug); // TODO: move outside of loop (need to make loop independent of state_)
      state_.setP(P_aug);

      // Add to list of estimated contact positions
      estimated_contact_positions_.insert(pair<int,int> (it->id, startIndex));
    }
  }
}


// Create Observation from vector of landmark measurements
void InEKF::CorrectLandmarks(const vectorLandmarks& measured_landmarks) {
  ocs2::vector_t Z, Y, b;
  ocs2::matrix_t H, N, PI;
  vectorLandmarks new_landmarks;
  vector<int> used_landmark_ids;

  for(vectorLandmarksIterator it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
    // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
    if(find(used_landmark_ids.begin(), used_landmark_ids.end(), it->id) != used_landmark_ids.end()) { 
      cout << "Duplicate landmark ID detected! Skipping measurement.\n";
      continue; 
    } 
    else { 
      used_landmark_ids.push_back(it->id); 
    }
    // See if we can find id in prior_landmarks or estimated_landmarks
    mapIntVector3dIterator it_prior = prior_landmarks_.find(it->id);
    map<int,int>::iterator it_estimated = estimated_landmarks_.find(it->id);
    if(it_prior!=prior_landmarks_.end()) {
      // Found in prior landmark set
      const int dimX = state_.dimX();
      const int dimTheta = state_.dimTheta();
      const int dimP = state_.dimP();
      int startIndex;

      // Fill out H
      startIndex = H.rows();
      H.conservativeResize(startIndex+3, dimP);
      H.block(startIndex,0,3,dimP).setZero();
      if(state_.getStateType() == StateType::WorldCentric) {
          H.block(startIndex,0,3,3) = skew(it_prior->second); // skew(p_wl)
          H.block(startIndex,6,3,3) = -matrix3_t::Identity(); // -I    
      } 
      else {
          H.block(startIndex,0,3,3) = skew(-it_prior->second); // -skew(p_wl)
          H.block(startIndex,6,3,3) = matrix3_t::Identity(); // I    
      }

      // Fill out N
      startIndex = N.rows();
      N.conservativeResize(startIndex+3, startIndex+3);
      N.block(startIndex,0,3,startIndex).setZero();
      N.block(0,startIndex,startIndex,3).setZero();
      N.block(startIndex,startIndex,3,3) = state_.getWorldRotation() * it->covariance * state_.getWorldRotation().transpose();

      // Fill out Z
      startIndex = Z.rows();
      Z.conservativeResize(startIndex+3, Eigen::NoChange);
      const auto& R = state_.getRotation();
      const auto& p = state_.getPosition();
      const auto& l = state_.getVector(it_estimated->second);  
      if(state_.getStateType() == StateType::WorldCentric) {
          Z.segment(startIndex,3) = R*it->position - (l - it_prior->second); 
      } 
      else {
          Z.segment(startIndex,3) = R.transpose()*(it->position - (p - it_prior->second)); 
      }
    } 
    else if(it_estimated!=estimated_landmarks_.end()) {;
      // Found in estimated landmark set
      const int dimX = state_.dimX();
      const int dimTheta = state_.dimTheta();
      const int dimP = state_.dimP();
      int startIndex;

      // Fill out H
      startIndex = H.rows();
      H.conservativeResize(startIndex+3, dimP);
      H.block(startIndex,0,3,dimP).setZero();
      if(state_.getStateType() == StateType::WorldCentric) {
          H.block(startIndex,6,3,3) = -matrix3_t::Identity(); // -I
          H.block(startIndex,3*it_estimated->second-dimTheta,3,3) = matrix3_t::Identity(); // I
      } 
      else {
          H.block(startIndex,6,3,3) = matrix3_t::Identity(); // I
          H.block(startIndex,3*it_estimated->second-dimTheta,3,3) = -matrix3_t::Identity(); // -I
      }

      // Fill out N
      startIndex = N.rows();
      N.conservativeResize(startIndex+3, startIndex+3);
      N.block(startIndex,0,3,startIndex).setZero();
      N.block(0,startIndex,startIndex,3).setZero();
      N.block(startIndex,startIndex,3,3).noalias() = state_.getWorldRotation() * it->covariance * state_.getWorldRotation().transpose();

      // Fill out Z
      startIndex = Z.rows();
      Z.conservativeResize(startIndex+3, Eigen::NoChange);
      const auto& R = state_.getRotation();
      const auto& p = state_.getPosition();
      const auto& l = state_.getVector(it_estimated->second);  
      if(state_.getStateType() == StateType::WorldCentric) {
          Z.segment(startIndex,3).noalias() = R*it->position - (l - p); 
      } 
      else {
          Z.segment(startIndex,3).noalias() = R.transpose()*(it->position - (p - l)); 
      }
    } 
    else {
      // First time landmark as been detected (add to list for later state augmentation)
      new_landmarks.push_back(*it);
    }
  }

  // Correct state using stacked observation
  if(Z.rows()>0) {
    if(state_.getStateType() == StateType::WorldCentric) {
      this->CorrectRightInvariant(Z,H,N);
    } 
    else {
      this->CorrectLeftInvariant(Z,H,N);
    }
  }

    // Augment state with newly detected landmarks
  if(new_landmarks.size() > 0) {
    ocs2::matrix_t X_aug = state_.getX(); 
    ocs2::matrix_t P_aug = state_.getP();
    for(vectorLandmarksIterator it=new_landmarks.begin(); it!=new_landmarks.end(); ++it) {
      // Initialize new landmark mean
      const int startIndex = X_aug.rows();
      X_aug.conservativeResize(startIndex+1, startIndex+1);
      X_aug.block(startIndex,0,1,startIndex).setZero();
      X_aug.block(0,startIndex,startIndex,1).setZero();
      X_aug(startIndex, startIndex) = 1;
      X_aug.block(0,startIndex,3,1) = state_.getPosition() + state_.getRotation()*it->position;

      // Initialize new landmark covariance - TODO:speed up
      ocs2::matrix_t F = ocs2::matrix_t::Zero(state_.dimP()+3,state_.dimP()); 
      F.block(0,0,state_.dimP()-state_.dimTheta(),state_.dimP()-state_.dimTheta()).setIdentity(); // for old X
      F.block(state_.dimP()-state_.dimTheta()+3,state_.dimP()-state_.dimTheta(),state_.dimTheta(),state_.dimTheta()).setIdentity(); // for theta
      ocs2::matrix_t G = ocs2::matrix_t::Zero(F.rows(),3);
      // Blocks for new landmark
      if(error_type_==ErrorType::RightInvariant) {
        F.block(state_.dimP()-state_.dimTheta(),6,3,3) = matrix3_t::Identity(); 
        G.block(G.rows()-state_.dimTheta()-3,0,3,3) = state_.getRotation();
      } else {
        F.block(state_.dimP()-state_.dimTheta(),6,3,3) = matrix3_t::Identity(); 
        F.block(state_.dimP()-state_.dimTheta(),0,3,3) = skew(-it->position); 
        G.block(G.rows()-state_.dimTheta()-3,0,3,3) = matrix3_t::Identity();
      }
      P_aug = (F*P_aug*F.transpose() + G*it->covariance*G.transpose()).eval();

      // Update state and covariance
      state_.setX(X_aug);
      state_.setP(P_aug);

      // Add to list of estimated landmarks
      estimated_landmarks_.insert(pair<int,int> (it->id, startIndex));
    }
  }
}


// Remove landmarks by IDs
void InEKF::RemoveLandmarks(const int landmark_id) {
    // Search for landmark in state
  map<int,int>::iterator it = estimated_landmarks_.find(landmark_id);
  if(it!=estimated_landmarks_.end()) {
    // Get current X and P
    ocs2::matrix_t X_rem = state_.getX(); 
    ocs2::matrix_t P_rem = state_.getP();
    // Remove row and column from X
    removeRowAndColumn(X_rem, it->second);
    // Remove 3 rows and columns from P
    int startIndex = 3 + 3*(it->second-3);
    removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
    removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
    removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
    // Update all indices for estimated_landmarks and estimated_contact_positions (TODO: speed this up)
    for(map<int,int>::iterator it2=estimated_landmarks_.begin(); it2!=estimated_landmarks_.end(); ++it2) {
      if(it2->second > it->second) it2->second -= 1;
    }
    for(map<int,int>::iterator it2=estimated_contact_positions_.begin(); it2!=estimated_contact_positions_.end(); ++it2) {
      if(it2->second > it->second) it2->second -= 1;
    }
    // Remove from list of estimated landmark positions (after we are done with iterator)
    estimated_landmarks_.erase(it->first);
    // Update state and covariance
    state_.setX(X_rem);
    state_.setP(P_rem);   
  }
}


// Remove landmarks by IDs
void InEKF::RemoveLandmarks(const std::vector<int>& landmark_ids) {
  // Loop over landmark_ids and remove
  for(int  i = 0;  i < landmark_ids.size(); ++i) {
    this->RemoveLandmarks(landmark_ids[i]);
  }
}


// Keep landmarks by IDs
void InEKF::KeepLandmarks(const std::vector<int>& landmark_ids) {
  std::cout << std::endl;
  // Loop through estimated landmarks removing ones not found in the list
  std::vector<int> ids_to_erase;
  for(map<int,int>::iterator it=estimated_landmarks_.begin(); it!=estimated_landmarks_.end(); ++it) {
    std::vector<int>::const_iterator it_found = find(landmark_ids.begin(), landmark_ids.end(), it->first);
    if(it_found==landmark_ids.end()) {
      // Get current X and P
      ocs2::matrix_t X_rem = state_.getX(); 
      ocs2::matrix_t P_rem = state_.getP();
      // Remove row and column from X
      removeRowAndColumn(X_rem, it->second);
      // Remove 3 rows and columns from P
      int startIndex = 3 + 3*(it->second-3);
      removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
      removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
      removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
      // Update all indices for estimated_landmarks and estimated_contact_positions (TODO: speed this up)
      for(map<int,int>::iterator it2=estimated_landmarks_.begin(); it2!=estimated_landmarks_.end(); ++it2) {
        if(it2->second > it->second) it2->second -= 1;
      }
      for(map<int,int>::iterator it2=estimated_contact_positions_.begin(); it2!=estimated_contact_positions_.end(); ++it2) {
        if(it2->second > it->second) it2->second -= 1;
      }
      // Add to list of ids to erase
      ids_to_erase.push_back(it->first);
      // Update state and covariance
      state_.setX(X_rem);
      state_.setP(P_rem);   
    }
  }
  // Remove from list of estimated landmark positions (after we are done with iterator)
  for(int  i = 0;  i < ids_to_erase.size(); ++i) {
    estimated_landmarks_.erase(ids_to_erase[i]);
  }
}


// Remove prior landmarks by IDs
void InEKF::RemovePriorLandmarks(const int landmark_id) {
  // Search for landmark in state
  mapIntVector3dIterator it = prior_landmarks_.find(landmark_id);
  if(it!=prior_landmarks_.end()) { 
    // Remove from list of estimated landmark positions
    prior_landmarks_.erase(it->first);
  }
}


// Remove prior landmarks by IDs
void InEKF::RemovePriorLandmarks(const std::vector<int>& landmark_ids) {
  // Loop over landmark_ids and remove
  for(int  i = 0;  i < landmark_ids.size(); ++i) {
    this->RemovePriorLandmarks(landmark_ids[i]);
  }
}


// Corrects state using magnetometer measurements (Right Invariant)
void InEKF::CorrectMagnetometer(const vector3_t& measured_magnetic_field, const matrix3_t& covariance) {
    // ocs2::vector_t Y, b;
    // ocs2::matrix_t H, N, PI;

    // // Get Rotation Estimate
    // matrix3_t R = state_.getRotation();

    // // Fill out observation data
    // int dimX = state_.dimX();
    // int dimTheta = state_.dimTheta();
    // int dimP = state_.dimP();

    // // Fill out Y
    // Y.conservativeResize(dimX, Eigen::NoChange);
    // Y.segment(0,dimX) = ocs2::vector_t::Zero(dimX);
    // Y.segment<3>(0) = measured_magnetic_field;

    // // Fill out b
    // b.conservativeResize(dimX, Eigen::NoChange);
    // b.segment(0,dimX) = ocs2::vector_t::Zero(dimX);
    // b.segment<3>(0) = magnetic_field_;

    // // Fill out H
    // H.conservativeResize(3, dimP);
    // H.block(0,0,3,dimP) = ocs2::matrix_t::Zero(3,dimP);
    // H.block<3,3>(0,0) = skew(magnetic_field_); 

    // // Fill out N
    // N.conservativeResize(3, 3);
    // N = R * covariance * R.transpose();

    // // Fill out PI      
    // PI.conservativeResize(3, dimX);
    // PI.block(0,0,3,dimX) = ocs2::matrix_t::Zero(3,dimX);
    // PI.block(0,0,3,3) = matrix3_t::Identity();
    

    // // Correct state using stacked observation
    // Observation obs(Y,b,H,N,PI);
    // if(!obs.empty()) {
    //     this->CorrectRightInvariant(obs);
    //     // cout << obs << endl;
    // }
}


// Observation of absolute position - GPS (Left-Invariant Measurement)
void InEKF::CorrectPosition(const vector3_t& measured_position, 
                            const matrix3_t& covariance, 
                            const vector3_t& indices) {
  // ocs2::vector_t Y, b;
  // ocs2::matrix_t H, N, PI;

  // // Fill out observation data
  // int dimX = state_.dimX();
  // int dimTheta = state_.dimTheta();
  // int dimP = state_.dimP();

  // // Fill out Y
  // Y.conservativeResize(dimX, Eigen::NoChange);
  // Y.segment(0,dimX) = ocs2::vector_t::Zero(dimX);
  // Y.segment<3>(0) = measured_position;
  // Y(4) = 1;       

  // // Fill out b
  // b.conservativeResize(dimX, Eigen::NoChange);
  // b.segment(0,dimX) = ocs2::vector_t::Zero(dimX);
  // b(4) = 1;       

  // // Fill out H
  // H.conservativeResize(3, dimP);
  // H.block(0,0,3,dimP) = ocs2::matrix_t::Zero(3,dimP);
  // H.block<3,3>(0,6) = matrix3_t::Identity(); 

  // // Fill out N
  // N.conservativeResize(3, 3);
  // N = covariance;

  // // Fill out PI      
  // PI.conservativeResize(3, dimX);
  // PI.block(0,0,3,dimX) = ocs2::matrix_t::Zero(3,dimX);
  // PI.block(0,0,3,3) = matrix3_t::Identity();

  // // Modify measurement based on chosen indices
  // const ocs2::scalar_t HIGH_UNCERTAINTY = 1e6;
  // vector3_t p = state_.getPosition();
  // if(!indices(0)) { 
  //   Y(0) = p(0);
  //   N(0,0) = HIGH_UNCERTAINTY;
  //   N(0,1) = 0;
  //   N(0,2) = 0;
  //   N(1,0) = 0;
  //   N(2,0) = 0;
  //   } 
  // if(!indices(1)) { 
  //   Y(1) = p(1);
  //   N(1,0) = 0;
  //   N(1,1) = HIGH_UNCERTAINTY;
  //   N(1,2) = 0;
  //   N(0,1) = 0;
  //   N(2,1) = 0;
  //   } 
  // if(!indices(2)) { 
  //   Y(2) = p(2);
  //   N(2,0) = 0;
  //   N(2,1) = 0;
  //   N(2,2) = HIGH_UNCERTAINTY;
  //   N(0,2) = 0;
  //   N(1,2) = 0;
  //   } 

  // // Correct state using stacked observation
  // Observation obs(Y,b,H,N,PI);
  // if(!obs.empty()) {
  //   this->CorrectLeftInvariant(obs);
  //   // cout << obs << endl;
  // }
}


// Observation of absolute z-position of contact points (Left-Invariant Measurement)
void InEKF::CorrectContactPosition(const int id, 
                                   const vector3_t& measured_contact_position, 
                                   const matrix3_t& covariance, 
                                   const vector3_t& indices) {
  ocs2::vector_t Z_full, Z;
  ocs2::matrix_t PI, H_full, N_full, H, N;

  // See if we can find id estimated_contact_positions
  map<int,int>::iterator it_estimated = estimated_contact_positions_.find(id);
  if(it_estimated!=estimated_contact_positions_.end()) { 

    // Fill out PI
    int startIndex;
    for(int  i = 0;  i < 3; ++i) {
      if(indices(i) != 0) {
        startIndex = PI.rows();
        PI.conservativeResize(startIndex+1, 3);  
        PI.template block<1,3>(startIndex,0).setZero();
        PI.coeffRef(startIndex,i) = 1;
      }  
    }
    if(PI.rows()==0) {
      return;
    }

    // Fill out observation data
    const int dimX = state_.dimX();
    const int dimTheta = state_.dimTheta();
    const int dimP = state_.dimP();

    // Get contact position
    const auto& d = state_.getVector(it_estimated->second);

    // Fill out H
    H_full = ocs2::matrix_t::Zero(3,dimP);
    H_full.block<3,3>(0,0) = -skew(d);
    H_full.block<3,3>(0,3*it_estimated->second-6) = matrix3_t::Identity();
    H.noalias() = PI*H_full;

    // Fill out N
    N_full = covariance;   
    N.noalias() = PI*N_full*PI.transpose();

    // Fill out Z
    Z_full = measured_contact_position - d; 
    Z.noalias() = PI*Z_full;

    // Correct
    this->CorrectRightInvariant(Z,H,N);
  }
}


void removeRowAndColumn(ocs2::matrix_t& M, int index) {
  const unsigned int dimX = M.cols();
  // cout << "Removing index: " << index<< endl;
  M.block(index,0,dimX-index-1,dimX) = M.bottomRows(dimX-index-1).eval();
  M.block(0,index,dimX,dimX-index-1) = M.rightCols(dimX-index-1).eval();
  M.conservativeResize(dimX-1,dimX-1);
}

} // end inekf namespace
