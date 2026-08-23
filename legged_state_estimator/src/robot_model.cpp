#include <legged_state_estimator/robot_model.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>


namespace legged_state_estimator 
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  pinocchio::Model RobotModel::buildFloatingBaseModel(const std::string& urdf_path) 
  {
    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), pin_model);
    return pin_model;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  RobotModel::RobotModel(const std::string& urdf_path, const int imu_frame,
    const std::vector<int>& contact_frames): RobotModel(
      buildFloatingBaseModel(urdf_path), imu_frame, contact_frames) { }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  RobotModel::RobotModel(const std::string& urdf_path, 
    const std::string& imu_frame,
    const std::vector<std::string>& contact_frames): RobotModel(
      buildFloatingBaseModel(urdf_path), imu_frame, contact_frames) { }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  RobotModel::RobotModel(const pinocchio::Model& pin_model, const int imu_frame,
    const std::vector<int>& contact_frames): contact_frames_(contact_frames),
      imu_frame_(imu_frame), model_(pin_model), data_(), q_(), v_(), a_(), tau_(), 
      jac_6d_() 
  {
    data_ = pinocchio::Data(model_);
    q_   = vector_t(model_.nq);
    v_   = vector_t(model_.nv);
    a_   = vector_t(model_.nv);
    tau_ = vector_t(model_.nv);
    for(int  i = 0;  i < contact_frames.size(); ++i) 
    {
      jac_6d_.push_back(ocs2::matrix_t::Zero(6, model_.nv));
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  RobotModel::RobotModel(const pinocchio::Model& pin_model, const std::string& imu_frame,
    const std::vector<std::string>& contact_frames): contact_frames_(), imu_frame_(), 
      model_(pin_model), data_(), q_(), v_(), a_(), tau_(), jac_6d_() 
  {
    data_ = pinocchio::Data(model_);
    q_   = vector_t(model_.nq);
    v_   = vector_t(model_.nv);
    a_   = vector_t(model_.nv);
    tau_ = vector_t(model_.nv);

    for(int  i = 0;  i < contact_frames.size(); ++i) 
    {
      jac_6d_.push_back(ocs2::matrix_t::Zero(6, model_.nv));
    }

    if(!model_.existFrame(imu_frame)) 
    {
      throw std::invalid_argument(
        "[RobotModel] invalid argument: IMU frame '" + imu_frame + "' does not exit!");
    }
    imu_frame_ = model_.getFrameId(imu_frame);

    if(contact_frames.size() > MAX_LEG_NUMBER) 
    {
      std::string error_message = "[RobotModel] invalid argment: contact_frames.size() must be smaller than " + MAX_LEG_NUMBER;
      throw std::invalid_argument(error_message);
    }

    if(contact_frames.size() < 2) 
    {
      std::string error_message = "[RobotModel] invalid argment: contact_frames.size() must be bigger than " + 2;
      throw std::invalid_argument(error_message);
    }

    contact_frames_.clear();

    for(const auto& e : contact_frames) 
    {
      if(!model_.existFrame(e)) 
      {
        throw std::invalid_argument(
          "[RobotModel] invalid argument: contact frame '" + e + "' does not exit!");
      }
      contact_frames_.push_back(model_.getFrameId(e));
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  RobotModel::RobotModel(): contact_frames_(), imu_frame_(), model_(), data_(), q_(), 
    v_(), a_(), tau_(), jac_6d_() { }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void RobotModel::updateLegKinematics(const vector_t& qJ, 
    const pinocchio::ReferenceFrame rf) 
  {
    updateKinematics(vector3_t::Zero(), 
      Eigen::Quaterniond::Identity().coeffs(), qJ, rf);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void RobotModel::updateLegKinematics(const vector_t& qJ, 
    const vector_t& dqJ, const pinocchio::ReferenceFrame rf) 
  {
    updateKinematics(vector3_t::Zero(), 
      Eigen::Quaterniond::Identity().coeffs(), 
      vector3_t::Zero(), vector3_t::Zero(), qJ, dqJ, rf);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void RobotModel::updateKinematics(const vector3_t& base_pos, 
    const vector4_t& base_quat, const vector_t& qJ, 
    const pinocchio::ReferenceFrame rf) 
  {
    q_.template head<3>()     = base_pos;
    q_.template segment<4>(3) = base_quat;
    q_.tail(model_.nq-7) = qJ;
    pinocchio::normalize(model_, q_);
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    for(int  i = 0;  i < contact_frames_.size(); ++i) 
    {
      pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], rf, jac_6d_[i]);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void RobotModel::updateKinematics(const vector3_t& base_pos, 
    const vector4_t& base_quat, const vector3_t& base_linear_vel, 
    const vector3_t& base_angular_vel, const vector_t& qJ, 
    const vector_t& dqJ, const pinocchio::ReferenceFrame rf) 
  {
    q_.template head<3>()     = base_pos;
    q_.template segment<4>(3) = base_quat;
    v_.template head<3>()     = base_linear_vel;
    v_.template segment<3>(3) = base_angular_vel;
    q_.tail(model_.nq-7) = qJ;
    v_.tail(model_.nv-6) = dqJ;
    pinocchio::normalize(model_, q_);
    pinocchio::forwardKinematics(model_, data_, q_, v_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    for(int  i = 0;  i < contact_frames_.size(); ++i) 
    {
      pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], rf, jac_6d_[i]);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void RobotModel::updateLegDynamics(const vector_t& qJ, 
    const vector_t& dqJ) 
  {
    updateDynamics(vector3_t::Zero(), 
      Eigen::Quaterniond::Identity().coeffs(), 
      vector3_t::Zero(), vector3_t::Zero(), 
      vector3_t::Zero(), vector3_t::Zero(), 
      qJ, dqJ, vector_t::Zero(nJ()));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void RobotModel::updateDynamics(const vector3_t& base_pos, 
    const vector4_t& base_quat, const vector3_t& base_linear_vel, 
    const vector3_t& base_angular_vel, const vector3_t& base_linear_acc, 
    const vector3_t& base_angular_acc, const vector_t& qJ, 
    const vector_t& dqJ, const vector_t& ddqJ) 
  {
    q_.template head<3>()     = base_pos;
    q_.template segment<4>(3) = base_quat;
    v_.template head<3>()     = base_linear_vel;
    v_.template segment<3>(3) = base_angular_vel;
    a_.template head<3>()     = base_linear_acc;
    a_.template segment<3>(3) = base_angular_acc;
    q_.tail(model_.nq-7) = qJ;
    v_.tail(model_.nv-6) = dqJ;
    a_.tail(model_.nv-6) = ddqJ;
    tau_ = pinocchio::rnea(model_, data_, q_, v_, a_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& RobotModel::getBasePosition() const 
  {
    return data_.oMf[imu_frame_].translation();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const matrix3_t& RobotModel::getBaseRotation() const 
  {
    return data_.oMf[imu_frame_].rotation();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& RobotModel::getContactPosition(const int contact_id) const 
  {
    assert(contact_id >= 0);
    assert(contact_id < contact_frames_.size());
    return data_.oMf[contact_frames_[contact_id]].translation();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const matrix3_t& RobotModel::getContactRotation(const int contact_id) const 
  {
    assert(contact_id >= 0);
    assert(contact_id < contact_frames_.size());
    return data_.oMf[contact_frames_[contact_id]].rotation();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const Eigen::Block<const ocs2::matrix_t> RobotModel::getContactJacobian(const int contact_id) const 
  {
    assert(contact_id >= 0);
    assert(contact_id < contact_frames_.size());
    return jac_6d_[contact_id].topRows(3);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const Eigen::Block<const ocs2::matrix_t> RobotModel::getJointContactJacobian(
    const int contact_id) const 
  {
    assert(contact_id >= 0);
    assert(contact_id < contact_frames_.size());
    return jac_6d_[contact_id].topRightCorner(3, nJ());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector_t& RobotModel::getInverseDynamics() const 
  {
    return tau_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const Eigen::VectorBlock<const vector_t> RobotModel::getJointInverseDynamics() const 
  {
    return tau_.tail(nJ());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const std::vector<int>& RobotModel::getContactFrames() const 
  {
    return contact_frames_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  int RobotModel::nq() const 
  {
    return model_.nq;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  int RobotModel::nv() const 
  {
    return model_.nv;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  int RobotModel::nJ() const 
  {
    return model_.nv-6;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  int RobotModel::numContacts() const 
  {
    return contact_frames_.size();
  }
} // namespace legged_state_estimator
