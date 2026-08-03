/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   lie_group.h
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#ifndef __LEGGED_STATE_ESTIMATOR_LIEGROUP___
#define __LEGGED_STATE_ESTIMATOR_LIEGROUP___
#include <iostream>
#include <cmath>

#include <Eigen/Core>

namespace legged_state_estimator {

long int factorial(const int n);
ocs2::matrix3_t skew(const ocs2::vector3_t& v);
ocs2::matrix3_t Gamma_SO3(const ocs2::vector3_t& w, const int n,
                          const ocs2::scalar_t exp_map_tol=1.0e-10);
ocs2::matrix3_t Exp_SO3(const ocs2::vector3_t& w);
ocs2::matrix3_t LeftJacobian_SO3(const ocs2::vector3_t& w);
ocs2::matrix3_t RightJacobian_SO3(const ocs2::vector3_t& w);
ocs2::matrix_t Exp_SEK3(const ocs2::vector_t& v, 
                         const ocs2::scalar_t exp_map_tol=1.0e-10);
ocs2::matrix_t Adjoint_SEK3(const ocs2::matrix_t& X);

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_LIEGROUP___