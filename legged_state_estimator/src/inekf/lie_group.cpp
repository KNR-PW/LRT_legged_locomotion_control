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
                          const ocs2::scalar_t /*exp_map_tol*/) {
  // Computes mth integral of the exponential map: \Gamma_m = \sum_{n=0}^{\infty} \dfrac{1}{(n+m)!} (w^\wedge)^n
  // Since A^3 = -theta^2*A, it reduces to: \Gamma_m = I/m! + alpha_m*A + beta_m*A^2
  // (exp_map_tol is no longer needed, small angles are handled by the series below)
  assert(m>=0);
  const ocs2::scalar_t theta2 = w.squaredNorm();
  const ocs2::scalar_t mfactorial = factorial(m);
  ocs2::scalar_t alpha, beta;
  if(theta2 < 0.0625) {
    // Series for theta < 0.25 (closed form loses precision there):
    // alpha_m = sum_k (-theta^2)^k/(2k+1+m)!,  beta_m = sum_k (-theta^2)^k/(2k+2+m)!
    ocs2::scalar_t alpha_term = 1.0/(mfactorial*(m+1));
    ocs2::scalar_t beta_term = alpha_term/(m+2);
    alpha = 0.0;
    beta = 0.0;
    for(int k=0; k<8; ++k) {
      alpha += alpha_term;
      beta += beta_term;
      alpha_term *= -theta2/((2*k+2+m)*(2*k+3+m));
      beta_term *= -theta2/((2*k+3+m)*(2*k+4+m));
    }
  }
  else {
    // Closed form via recursion from the exp map: alpha_m = beta_{m-1}, beta_m = (1/m! - alpha_{m-1})/theta^2
    const ocs2::scalar_t theta = std::sqrt(theta2);
    alpha = std::sin(theta)/theta;
    beta = (1-std::cos(theta))/theta2;
    ocs2::scalar_t jfactorial = 1.0;
    for(int j=1; j<=m; ++j) {
      jfactorial *= j;
      const ocs2::scalar_t alpha_prev = alpha;
      alpha = beta;
      beta = (1.0/jfactorial - alpha_prev)/theta2;
    }
  }
  const matrix3_t A = skew(w);
  matrix3_t G = alpha*A + beta*(A*A);
  G.diagonal().array() += 1.0/mfactorial;
  return G;
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