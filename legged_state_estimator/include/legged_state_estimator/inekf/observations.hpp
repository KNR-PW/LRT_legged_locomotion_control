/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   observations.hpp
 *  @author Ross Hartley
 *  @brief  Header file for Observations
 *  @date   December 03, 2018
 **/

#ifndef __LEGGED_STATE_ESTIMATOR_OBSERVATIONS__
#define __LEGGED_STATE_ESTIMATOR_OBSERVATIONS__

#include <iostream>
#include <map>

#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

// Simple class to hold general observations 
struct Observation {
public:
  Observation(ocs2::vector_t& Y, ocs2::vector_t& b, ocs2::matrix_t& H, 
              ocs2::matrix_t& N, ocs2::matrix_t& PI);
  Observation() = default;

  ~Observation() = default;

  Observation(const Observation&) = default;
  Observation& operator=(const Observation&) = default;
  Observation(Observation&&) noexcept = default;
  Observation& operator=(Observation&&) noexcept = default;

  bool empty();

  ocs2::vector_t Y;
  ocs2::vector_t b;
  ocs2::matrix_t H;
  ocs2::matrix_t N;
  ocs2::matrix_t PI;

  friend std::ostream& operator<<(std::ostream& os, const Observation& o);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Kinematics {
public:
  Kinematics(const int id_in, const Eigen::Matrix4d& pose_in, 
             const Eigen::Matrix<ocs2::scalar_t,6,6>& covariance_in) 
    : id(id_in), pose(pose_in), covariance(covariance_in) { }
  Kinematics(const int id_in, const matrix3_t& rotation_in, 
             const vector3_t& position_in, 
             const Eigen::Matrix<ocs2::scalar_t,6,6>& covariance_in) 
    : id(id_in), pose(Eigen::Matrix4d::Identity()), covariance(covariance_in) {
        setContactRotation(rotation_in);
        setContactPosition(position_in);
  }
  Kinematics() = default;

  ~Kinematics() = default;

  Kinematics(const Kinematics&) = default;
  Kinematics& operator=(const Kinematics&) = default;
  Kinematics(Kinematics&&) noexcept = default;
  Kinematics& operator=(Kinematics&&) noexcept = default;

  void setContactPosition(const vector3_t& position_in) {
      pose.template block<3,1>(0,3) = position_in;
  }

  void setContactRotation(const matrix3_t& rotation_in) {
      pose.template block<3,3>(0,0) = rotation_in;
  }

  void setContactPositionCovariance(const matrix3_t& covariance_in) {
      covariance.template bottomRightCorner<3,3>() = covariance_in;
  }

  int id;
  Eigen::Matrix4d pose;
  Eigen::Matrix<ocs2::scalar_t,6,6> covariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Landmark {
public:
  Landmark(const int id_in, const vector3_t& position_in, 
            const matrix3_t& covariance_in) 
    : id(id_in), position(position_in), covariance(covariance_in) { }
  Landmark() = default;

  ~Landmark() = default;

  Landmark(const Landmark&) = default;
  Landmark& operator=(const Landmark&) = default;
  Landmark(Landmark&&) noexcept = default;
  Landmark& operator=(Landmark&&) noexcept = default;

  int id;
  vector3_t position;
  matrix3_t covariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** A map with an integer as key and a vector3_t as value. */
using mapIntVector3d = std::map<int,vector3_t, std::less<int>, Eigen::aligned_allocator<std::pair<const int,vector3_t>>>;
using mapIntVector3dIterator = std::map<int,vector3_t, std::less<int>, Eigen::aligned_allocator<std::pair<const int,vector3_t>>>::const_iterator;

/** A vector of Kinematics. */
using vectorKinematics = std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>>;
using vectorKinematicsIterator = std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>>::const_iterator;

/** A vector of Landmark. */
using vectorLandmarks = std::vector<Landmark, Eigen::aligned_allocator<Landmark>>;
using vectorLandmarksIterator = std::vector<Landmark, Eigen::aligned_allocator<Landmark>>::const_iterator;

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_OBSERVATIONS__