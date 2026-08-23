/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.cpp
 *  @author Ross Hartley
 *  @brief  Source file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#include "legged_state_estimator/inekf/lie_group.hpp"


namespace legged_state_estimator {

using namespace std;

long int factorial(const int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

matrix3_t skew(const vector3_t& v) {
  // Convert vector to skew-symmetric matrix
  matrix3_t M = matrix3_t::Zero();
  M <<           0, -v.coeff(2),  v.coeff(1),
        v.coeff(2),           0, -v.coeff(0), 
       -v.coeff(1),  v.coeff(0),           0;
  return M;
}

matrix3_t Gamma_SO3(const vector3_t& w, const int m, 
                          const ocs2::scalar_t exp_map_tol) {
  // Computes mth integral of the exponential map: \Gamma_m = \sum_{n=0}^{\infty} \dfrac{1}{(n+m)!} (w^\wedge)^n
  assert(m>=0);
  const ocs2::scalar_t theta = w.norm();
  if(theta < exp_map_tol) {
      return (1.0/factorial(m))*matrix3_t::Identity(); // TODO: There is a better small value approximation for exp() given in Trawny p.19
  } 
  const matrix3_t A = skew(w);
  const ocs2::scalar_t theta2 =  theta*theta;

  // Closed form solution for the first 3 cases
  switch (m) {
    case 0: // Exp map of SO(3)
      return matrix3_t::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/theta2)*A*A;
    
    case 1: // Left Jacobian of SO(3)
      // eye(3) - A*(1/theta^2) * (R - eye(3) - A);
      // eye(3) + (1-cos(theta))/theta^2 * A + (theta-sin(theta))/theta^3 * A^2;
      return matrix3_t::Identity() + ((1-cos(theta))/theta2)*A + ((theta-sin(theta))/(theta2*theta))*A*A;

    case 2: 
      // 0.5*eye(3) - (1/theta^2) * (R - eye(3) - A - 0.5*A^2);
      // 0.5*eye(3) + (theta-sin(theta))/theta^3 * A + (2*(cos(theta)-1) + theta^2)/(2*theta^4) * A^2
      return 0.5*matrix3_t::Identity() + (theta-sin(theta))/(theta2*theta)*A + (theta2 + 2*cos(theta)-2)/(2*theta2*theta2)*A*A;

    default: // General case 
      const matrix3_t R = matrix3_t::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/theta2)*A*A;
      matrix3_t S = matrix3_t::Identity();
      matrix3_t Ak = matrix3_t::Identity();
      long int kfactorial = 1;
      for(int k=1; k<=m; ++k) {
        kfactorial = kfactorial*k;
        Ak = (Ak*A).eval();
        S = (S + (1.0/kfactorial)*Ak).eval();
      }
      if(m==0) { 
          return R;
      } 
      else if(m%2){ // odd 
          return (1.0/kfactorial)*matrix3_t::Identity() + (pow(-1,(m+1)/2)/pow(theta,m+1))*A * (R - S);
      } 
      else { // even
          return (1.0/kfactorial)*matrix3_t::Identity() + (pow(-1,m/2)/pow(theta,m)) * (R - S);
      }
  }
}

matrix3_t Exp_SO3(const vector3_t& w) {
  // Computes the vectorized exponential map for SO(3)
  return Gamma_SO3(w, 0);
}

matrix3_t LeftJacobian_SO3(const vector3_t& w) {
  // Computes the Left Jacobian of SO(3)
  return Gamma_SO3(w, 1);
}

matrix3_t RightJacobian_SO3(const vector3_t& w) {
  // Computes the Right Jacobian of SO(3)
  return Gamma_SO3(-w, 1);
}

ocs2::matrix_t Exp_SEK3(const ocs2::vector_t& v,  const ocs2::scalar_t exp_map_tol) {
  // Computes the vectorized exponential map for SE_K(3)
  const int K = (v.size()-3)/3;
  ocs2::matrix_t X = ocs2::matrix_t::Identity(3+K,3+K);
  matrix3_t R;
  matrix3_t Jl;
  const auto& w = v.head(3);
  ocs2::scalar_t theta = w.norm();
  if(theta < exp_map_tol) {
    R = matrix3_t::Identity();
    Jl = matrix3_t::Identity();
  } else {
    const matrix3_t A = skew(w);
    const ocs2::scalar_t theta2 = theta*theta;
    const ocs2::scalar_t stheta = sin(theta);
    const ocs2::scalar_t ctheta = cos(theta);
    const ocs2::scalar_t oneMinusCosTheta2 = (1-ctheta)/(theta2);
    const matrix3_t A2 = A*A;
    R.noalias() = matrix3_t::Identity() 
                   + (stheta/theta) * A + oneMinusCosTheta2 * A2;
    Jl.noalias() = matrix3_t::Identity() 
                   + oneMinusCosTheta2*A + ((theta-stheta)/(theta2*theta)) * A2;
  }
  X.block<3,3>(0,0) = R;
  for(int  i = 0;  i < K; ++i) {
      X.block<3,1>(0,3+i).noalias() = Jl * v.segment<3>(3+3*i);
  }
  return X;
}

ocs2::matrix_t Adjoint_SEK3(const ocs2::matrix_t& X) {
  // Compute Adjoint(X) for X in SE_K(3)
  const int K = X.cols()-3;
  ocs2::matrix_t Adj = ocs2::matrix_t::Zero(3+3*K, 3+3*K);
  const auto& R = X.block<3,3>(0,0);
  Adj.block<3,3>(0,0) = R;
  for(int  i = 0;  i < K; ++i) {
    Adj.block<3,3>(3+3*i,3+3*i) = R;
    Adj.block<3,3>(3+3*i,0).noalias() = skew(X.block<3,1>(0,3+i)) * R;
  }
  return Adj;
}


} // namespace legged_state_estimator 