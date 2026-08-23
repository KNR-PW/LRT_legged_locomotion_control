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

#ifndef __LEGGED_STATE_ESTIMATOR_LIEGROUP__
#define __LEGGED_STATE_ESTIMATOR_LIEGROUP__

#include <iostream>
#include <cmath>

#include "legged_state_estimator/types.hpp"

namespace legged_state_estimator {

long int factorial(const int n);
matrix3_t skew(const vector3_t& v);
matrix3_t Gamma_SO3(const vector3_t& w, const int n,
                          const ocs2::scalar_t exp_map_tol=1.0e-10);
matrix3_t Exp_SO3(const vector3_t& w);
matrix3_t LeftJacobian_SO3(const vector3_t& w);
matrix3_t RightJacobian_SO3(const vector3_t& w);
ocs2::matrix_t Exp_SEK3(const ocs2::vector_t& v, 
                         const ocs2::scalar_t exp_map_tol=1.0e-10);
ocs2::matrix_t Adjoint_SEK3(const ocs2::matrix_t& X);

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_LIEGROUP__