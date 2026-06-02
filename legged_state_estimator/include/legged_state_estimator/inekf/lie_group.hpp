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

#ifndef LEGGED_STATE_ESTIMATOR_LIEGROUP_HPP_
#define LEGGED_STATE_ESTIMATOR_LIEGROUP_HPP_
#include <iostream>
#include <cmath>

#include <Eigen/Core>

namespace legged_state_estimator {

long int factorial(const int n);
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d Gamma_SO3(const Eigen::Vector3d& w, const int n,
                          const double exp_map_tol=1.0e-10);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
Eigen::Matrix3d LeftJacobian_SO3(const Eigen::Vector3d& w);
Eigen::Matrix3d RightJacobian_SO3(const Eigen::Vector3d& w);
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v, 
                         const double exp_map_tol=1.0e-10);
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_LIEGROUP_HPP_