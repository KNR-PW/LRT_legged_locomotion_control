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

#ifndef LEGGED_STATE_ESTIMATOR_OBSERVATIONS_HPP_
#define LEGGED_STATE_ESTIMATOR_OBSERVATIONS_HPP_

#include <iostream>
#include <vector>
#include <map>

#include <Eigen/Core>


namespace legged_state_estimator {

// Simple class to hold general observations 
struct Observation {
public:
  Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, 
              Eigen::MatrixXd& N, Eigen::MatrixXd& PI);
  Observation() = default;

  ~Observation() = default;

  Observation(const Observation&) = default;
  Observation& operator=(const Observation&) = default;
  Observation(Observation&&) noexcept = default;
  Observation& operator=(Observation&&) noexcept = default;

  bool empty();

  Eigen::VectorXd Y;
  Eigen::VectorXd b;
  Eigen::MatrixXd H;
  Eigen::MatrixXd N;
  Eigen::MatrixXd PI;

  friend std::ostream& operator<<(std::ostream& os, const Observation& o);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Kinematics {
public:
  Kinematics(const int id_in, const Eigen::Matrix4d& pose_in, 
             const Eigen::Matrix<double,6,6>& covariance_in) 
    : id(id_in), pose(pose_in), covariance(covariance_in) { }
  Kinematics(const int id_in, const Eigen::Matrix3d& rotation_in, 
             const Eigen::Vector3d& position_in, 
             const Eigen::Matrix<double,6,6>& covariance_in) 
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

  void setContactPosition(const Eigen::Vector3d& position_in) {
      pose.template block<3,1>(0,3) = position_in;
  }

  void setContactRotation(const Eigen::Matrix3d& rotation_in) {
      pose.template block<3,3>(0,0) = rotation_in;
  }

  void setContactPositionCovariance(const Eigen::Matrix3d& covariance_in) {
      covariance.template bottomRightCorner<3,3>() = covariance_in;
  }

  int id;
  Eigen::Matrix4d pose;
  Eigen::Matrix<double,6,6> covariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct Landmark {
public:
  Landmark(const int id_in, const Eigen::Vector3d& position_in, 
            const Eigen::Matrix3d& covariance_in) 
    : id(id_in), position(position_in), covariance(covariance_in) { }
  Landmark() = default;

  ~Landmark() = default;

  Landmark(const Landmark&) = default;
  Landmark& operator=(const Landmark&) = default;
  Landmark(Landmark&&) noexcept = default;
  Landmark& operator=(Landmark&&) noexcept = default;

  int id;
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** A map with an integer as key and a Eigen::Vector3d as value. */
using mapIntVector3d = std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d>>>;
using mapIntVector3dIterator = std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d>>>::const_iterator;

/** A vector of Kinematics. */
using vectorKinematics = std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>>;
using vectorKinematicsIterator = std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>>::const_iterator;

/** A vector of Landmark. */
using vectorLandmarks = std::vector<Landmark, Eigen::aligned_allocator<Landmark>>;
using vectorLandmarksIterator = std::vector<Landmark, Eigen::aligned_allocator<Landmark>>::const_iterator;

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_OBSERVATIONS_HPP_