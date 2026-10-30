/******************************************************************************
BSD 3-Clause License

Copyright (c) 2019, Ross Hartley
Copyright (c) 2023, mayataka
Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2026
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __LEGGED_STATE_ESTIMATOR_ROBOT_MODEL__
#define __LEGGED_STATE_ESTIMATOR_ROBOT_MODEL__

#include <pinocchio/fwd.hpp>

#include <legged_state_estimator/types.hpp>

#include <pinocchio/multibody.hpp>

namespace legged_state_estimator 
{
  ///
  /// @class RobotModel
  /// @brief Dynamics and kinematics model of robots. Wraps pinocchio::Model and 
  /// pinocchio::Data. Includes contacts.
  ///
  class RobotModel 
  {
    public:
      ///
      /// @brief Constructs a robot model. Builds the Pinocchio robot model and data 
      /// from URDF. 
      /// @param[in] urdf_path Path to the URDF file.
      /// @param[in] imu_frames id of the IMU frame.
      /// @param[in] contact_frames Collection of the id of frames that can have 
      /// contacts with the environments. 
      ///
      RobotModel(const std::string& urdf_path, const int imu_frame, 
        const std::vector<int>& contact_frames);

      ///
      /// @brief Constructs a robot model. Builds the Pinocchio robot model and data
      /// from URDF. Base frame is the URDF root link.
      /// @param[in] urdf_path Path to the URDF file.
      /// @param[in] imu_frames Name of the IMU frame.
      /// @param[in] contact_frames Collection of the names of frames that can have
      /// contacts with the environments.
      ///
      RobotModel(const std::string& urdf_path, const std::string& imu_frame,
        const std::vector<std::string>& contact_frames);

      ///
      /// @brief Constructs a robot model. Builds the Pinocchio robot model and data
      /// from URDF.
      /// @param[in] urdf_path Path to the URDF file.
      /// @param[in] base_frame Name of the base frame. It must be connected to
      /// the URDF root link only by fixed joints. If empty, the URDF root link is used.
      /// @param[in] imu_frames Name of the IMU frame. It must be connected to
      /// the base frame only by fixed joints.
      /// @param[in] contact_frames Collection of the names of frames that can have
      /// contacts with the environments.
      ///
      RobotModel(const std::string& urdf_path, const std::string& base_frame,
        const std::string& imu_frame, const std::vector<std::string>& contact_frames);

      ///
      /// @brief Constructs a robot model from the Pinocchio robot model.
      /// @param[in] pin_model Pinocchio robot model.
      /// @param[in] imu_frames id of the IMU frame.
      /// @param[in] contact_frames Collection of the id of frames that can have 
      /// contacts with the environments. 
      ///
      RobotModel(const pinocchio::Model& pin_model, const int imu_frame, 
        const std::vector<int>& contact_frames);

      ///
      /// @brief Constructs a robot model from the Pinocchio robot model. Base
      /// frame is the frame of the root (free-flyer) joint.
      /// @param[in] pin_model Pinocchio robot model.
      /// @param[in] imu_frames Name of the IMU frame.
      /// @param[in] contact_frames Collection of the names of frames that can have
      /// contacts with the environments.
      ///
      RobotModel(const pinocchio::Model& pin_model, const std::string& imu_frame,
        const std::vector<std::string>& contact_frames);

      ///
      /// @brief Constructs a robot model from the Pinocchio robot model.
      /// @param[in] pin_model Pinocchio robot model.
      /// @param[in] base_frame Name of the base frame. It must be connected to
      /// the root (free-flyer) joint only by fixed joints. If empty, the frame of
      /// the root joint is used.
      /// @param[in] imu_frames Name of the IMU frame. It must be connected to
      /// the base frame only by fixed joints.
      /// @param[in] contact_frames Collection of the names of frames that can have
      /// contacts with the environments.
      ///
      RobotModel(const pinocchio::Model& pin_model, const std::string& base_frame,
        const std::string& imu_frame, const std::vector<std::string>& contact_frames);

      ///
      /// @brief Default constructor. 
      ///
      RobotModel();

      ///
      /// @brief Default destructor. 
      ///
      ~RobotModel() = default;

      RobotModel(const RobotModel&) = default;
      RobotModel& operator=(const RobotModel&) = default;
      RobotModel(RobotModel&&) noexcept = default;
      RobotModel& operator=(RobotModel&&) noexcept = default;

      ///
      /// @brief Updates leg kinemarics.
      /// @param[in] qJ Joint positions. Size must be RobotModel::nJ().
      /// @param[in] rf Reference frame of the kinematics. Default is
      /// pinocchio::LOCAL_WORLD_ALIGNED.
      /// @param[in] compute_jacobians If false, only frame placements are
      /// updated (contact Jacobians are not). Default is true.
      ///
      void updateLegKinematics(const ocs2::vector_t& qJ,
        const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED,
        const bool compute_jacobians=true);

      ///
      /// @brief Updates leg kinemarics.
      /// @param[in] qJ Joint positions. Size must be RobotModel::nJ().
      /// @param[in] dqJ Joint velocities. Size must be RobotModel::nJ().
      /// @param[in] rf Reference frame of the kinematics. Default is 
      /// pinocchio::LOCAL_WORLD_ALIGNED.
      ///
      void updateLegKinematics(const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
        const pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED);

      ///
      /// @brief Updates kinemarics.
      /// @param[in] base_pos Base position. 
      /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
      /// @param[in] qJ Joint positions. Size must be RobotModel::nJ().
      /// @param[in] rf Reference frame of the kinematics. Default is
      /// pinocchio::LOCAL_WORLD_ALIGNED.
      /// @param[in] compute_jacobians If false, only frame placements are
      /// updated (contact Jacobians are not). Default is true.
      ///
      void updateKinematics(const vector3_t& base_pos,
        const vector4_t& base_quat, const ocs2::vector_t& qJ,
        const pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED,
        const bool compute_jacobians = true);

      ///
      /// @brief Updates kinemarics.
      /// @param[in] base_pos Base position. 
      /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
      /// @param[in] base_linear_vel Base linear velocity expressed in the body 
      /// local coordinate. 
      /// @param[in] base_angular_vel Base angular velocity expressed in the body 
      /// local coordinate. 
      /// @param[in] qJ Joint positions. Size must be RobotModel::nJ().
      /// @param[in] dqJ Joint velocities. Size must be RobotModel::nJ().
      /// @param[in] rf Reference frame of the kinematics. Default is 
      /// pinocchio::LOCAL_WORLD_ALIGNED.
      ///
      void updateKinematics(const vector3_t& base_pos, 
        const vector4_t& base_quat, const vector3_t& base_linear_vel, 
        const vector3_t& base_angular_vel, const ocs2::vector_t& qJ, 
        const ocs2::vector_t& dqJ,
        const pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED);

      ///
      /// @brief Updates leg dynamics (base at rest).
      /// @param[in] qJ Joint positions. Size must be RobotModel::nJ().
      /// @param[in] dqJ Joint velocities. Size must be RobotModel::nJ().
      /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w),
      /// defines gravity direction relative to the base. Default is identity.
      ///
      void updateLegDynamics(const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ,
        const vector4_t& base_quat = vector4_t(0, 0, 0, 1));

      ///
      /// @brief Updates dynamics.
      /// @param[in] base_pos Base position. 
      /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
      /// @param[in] base_linear_vel Base linear velocity expressed in the body 
      /// local coordinate. 
      /// @param[in] base_angular_vel Base angular velocity expressed in the body 
      /// local coordinate. 
      /// @param[in] base_linear_vel Base linear acceleration expressed in the body 
      /// local coordinate. 
      /// @param[in] base_angular_vel Base angular acceleration expressed in the body 
      /// local coordinate. 
      /// @param[in] qJ Joint positions. Size must be RobotModel::nJ().
      /// @param[in] dqJ Joint velocities. Size must be RobotModel::nJ().
      /// @param[in] ddqJ Joint accelerations. Size must be RobotModel::nJ().
      ///
      void updateDynamics(const vector3_t& base_pos, 
        const vector4_t& base_quat, const vector3_t& base_linear_vel, 
        const vector3_t& base_angular_vel, const vector3_t& base_linear_acc, 
        const vector3_t& base_angular_acc, const ocs2::vector_t& qJ, 
        const ocs2::vector_t& dqJ, const ocs2::vector_t& ddqJ);

      ///
      /// @return const reference to the base frame position.
      ///
      const vector3_t& getBasePosition() const;

      ///
      /// @return const reference to the base frame orientation expressed by a rotation
      /// matrix.
      ///
      const matrix3_t& getBaseRotation() const;

      ///
      /// @return const reference to the IMU frame position.
      ///
      const vector3_t& getImuPosition() const;

      ///
      /// @return const reference to the IMU frame orientation expressed by a rotation
      /// matrix.
      ///
      const matrix3_t& getImuRotation() const;

      ///
      /// @return const reference to the (constant) IMU frame placement expressed in
      /// the base frame.
      ///
      const pinocchio::SE3& getImuPlacementInBase() const;

      ///
      /// @return const reference to the (constant) IMU frame placement expressed in
      /// the root (free-flyer) joint frame.
      ///
      const pinocchio::SE3& getImuPlacementInRoot() const;

      ///
      /// @return const reference to the (constant) base frame placement expressed in
      /// the root (free-flyer) joint frame.
      ///
      const pinocchio::SE3& getBasePlacementInRoot() const;

      ///
      /// @return const reference to the contact position. 
      ///
      const vector3_t& getContactPosition(const int contact_id) const;

      ///
      /// @return const reference to the contact orientation expressed by a rotation
      /// matrix. 
      ///
      const matrix3_t& getContactRotation(const int contact_id) const;

      ///
      /// @return const reference to the contact Jacobian with respect to the 
      /// generalized coordinate (base pos + base orn + joint positions). 
      /// Size is 3 x RobotModel::nv().
      ///
      const Eigen::Block<const ocs2::matrix_t> getContactJacobian(const int contact_id) const;

      ///
      /// @return const reference to the contact Jacobian with respect to the 
      /// joint positions. Size is 3 x RobotModel::nJ().
      ///
      const Eigen::Block<const ocs2::matrix_t> getJointContactJacobian(
        const int contact_id) const;

      ///
      /// @return const reference to the inverse dynamics. Size is RobotModel::nv().
      ///
      const ocs2::vector_t& getInverseDynamics() const;

      ///
      /// @return const reference to the inverse dynamics at the joint. Size is 
      /// RobotModel::nJ().
      /// 
      const Eigen::VectorBlock<const ocs2::vector_t> getJointInverseDynamics() const;

      ///
      /// @return const reference to the contact frames.
      ///
      const std::vector<int>& getContactFrames() const;

      ///
      /// @return Names of the joints (floating base joint excluded), in the
      /// order of the joint vectors (qJ, dqJ, tauJ).
      ///
      std::vector<std::string> getJointNames() const;

      ///
      /// @return Indices in the generalized velocity of the joints between the
      /// base and the contact frame (floating base joint excluded), ordered
      /// from the base to the contact.
      ///
      std::vector<int> getContactJointVelocityIndices(const int contact_id) const;

      ///
      /// @return Dimension of the configuration.
      ///
      int nq() const;

      ///
      /// @return Dimension of the generalized velocity.
      ///
      int nv() const;

      ///
      /// @return Number of the joints.
      ///
      int nJ() const;

      ///
      /// @return Number of the contacts.
      ///
      int numContacts() const;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      static pinocchio::Model buildFloatingBaseModel(const std::string& urdf_path);

    private:
      ///
      /// @brief Sets the base frame, checks that IMU and base frames are rigidly
      /// attached to the floating base and computes the IMU placement in the base frame.
      ///
      void initializeBaseFrame(const std::string& base_frame);

      pinocchio::Model model_;
      pinocchio::Data data_;
      ocs2::vector_t q_, v_, a_, tau_;
      std::vector<ocs2::matrix_t> jac_6d_;
      int imu_frame_;
      std::vector<int> contact_frames_;
      int base_frame_;
      pinocchio::SE3 imu_placement_in_base_;

  };
} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_ROBOT_MODEL__