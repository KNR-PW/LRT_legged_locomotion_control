#include "legged_state_estimator/robot_model.hpp"


namespace legged_state_estimator {

pinocchio::Model RobotModel::buildFloatingBaseModel(
    const std::string& urdf_path) {
  pinocchio::Model pin_model;
  pinocchio::urdf::buildModel(urdf_path, 
                              pinocchio::JointModelFreeFlyer(), pin_model);
  return pin_model;
}


RobotModel::RobotModel(const std::string& urdf_path, const int imu_frame,
                       const std::vector<int>& contact_frames) 
  : RobotModel(buildFloatingBaseModel(urdf_path), imu_frame, contact_frames) {
}


RobotModel::RobotModel(const std::string& urdf_path, 
                       const std::string& imu_frame,
                       const std::vector<std::string>& contact_frames) 
  : RobotModel(buildFloatingBaseModel(urdf_path), imu_frame, contact_frames) {
}


RobotModel::RobotModel(const pinocchio::Model& pin_model, const int imu_frame,
                       const std::vector<int>& contact_frames) 
  : contact_frames_(contact_frames),
    imu_frame_(imu_frame),
    model_(pin_model),
    data_(),
    q_(),
    v_(),
    a_(),
    tau_(),
    jac_6d_() {
  data_ = pinocchio::Data(model_);
  q_   = Eigen::VectorXd(model_.nq);
  v_   = Eigen::VectorXd(model_.nv);
  a_   = Eigen::VectorXd(model_.nv);
  tau_ = Eigen::VectorXd(model_.nv);
  for (int i=0; i<contact_frames.size(); ++i) {
    jac_6d_.push_back(Eigen::MatrixXd::Zero(6, model_.nv));
  }
}


RobotModel::RobotModel(const pinocchio::Model& pin_model,   
                       const std::string& imu_frame,
                       const std::vector<std::string>& contact_frames) 
  : contact_frames_(),
    imu_frame_(),
    model_(pin_model),
    data_(),
    q_(),
    v_(),
    a_(),
    tau_(),
    jac_6d_() {
  data_ = pinocchio::Data(model_);
  q_   = Eigen::VectorXd(model_.nq);
  v_   = Eigen::VectorXd(model_.nv);
  a_   = Eigen::VectorXd(model_.nv);
  tau_ = Eigen::VectorXd(model_.nv);
  for (int i=0; i<contact_frames.size(); ++i) {
    jac_6d_.push_back(Eigen::MatrixXd::Zero(6, model_.nv));
  }
  if (!model_.existFrame(imu_frame)) {
    throw std::invalid_argument(
        "[RobotModel] invalid argument: IMU frame '" + imu_frame + "' does not exit!");
  }
  imu_frame_ = model_.getFrameId(imu_frame);

  if (contact_frames.size() != 4) {
    throw std::invalid_argument(
        "[RobotModel] invalid argment: contact_frames.size() must be 4");
  }
  contact_frames_.clear();
  for (const auto& e : contact_frames) {
    if (!model_.existFrame(e)) {
      throw std::invalid_argument(
          "[RobotModel] invalid argument: contact frame '" + e + "' does not exit!");
    }
    contact_frames_.push_back(model_.getFrameId(e));
  }
}


RobotModel::RobotModel() 
  : contact_frames_(),
    imu_frame_(),
    model_(),
    data_(),
    q_(),
    v_(),
    a_(),
    tau_(),
    jac_6d_() {
}


void RobotModel::updateLegKinematics(const Eigen::VectorXd& qJ, 
                                     const pinocchio::ReferenceFrame rf) {
  updateKinematics(Eigen::Vector3d::Zero(), 
                   Eigen::Quaterniond::Identity().coeffs(), qJ, rf);
}


void RobotModel::updateLegKinematics(const Eigen::VectorXd& qJ, 
                                const Eigen::VectorXd& dqJ,
                                const pinocchio::ReferenceFrame rf) {
  updateKinematics(Eigen::Vector3d::Zero(), 
                   Eigen::Quaterniond::Identity().coeffs(), 
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
                   qJ, dqJ, rf);
}


void RobotModel::updateKinematics(const Eigen::Vector3d& base_pos, 
                                  const Eigen::Vector4d& base_quat, 
                                  const Eigen::VectorXd& qJ, 
                                  const pinocchio::ReferenceFrame rf) {
  q_.template head<3>()     = base_pos;
  q_.template segment<4>(3) = base_quat;
  q_.tail(model_.nq-7) = qJ;
  pinocchio::normalize(model_, q_);
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_);
  for (int i=0; i<contact_frames_.size(); ++i) {
    pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], rf, jac_6d_[i]);
  }
}


void RobotModel::updateKinematics(const Eigen::Vector3d& base_pos, 
                                  const Eigen::Vector4d& base_quat, 
                                  const Eigen::Vector3d& base_linear_vel, 
                                  const Eigen::Vector3d& base_angular_vel, 
                                  const Eigen::VectorXd& qJ, 
                                  const Eigen::VectorXd& dqJ, 
                                  const pinocchio::ReferenceFrame rf) {
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
  for (int i=0; i<contact_frames_.size(); ++i) {
    pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], rf, jac_6d_[i]);
  }
}


void RobotModel::updateLegDynamics(const Eigen::VectorXd& qJ, 
                                   const Eigen::VectorXd& dqJ) {
  updateDynamics(Eigen::Vector3d::Zero(), 
                 Eigen::Quaterniond::Identity().coeffs(), 
                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
                 qJ, dqJ, Eigen::VectorXd::Zero(nJ()));
}


void RobotModel::updateDynamics(const Eigen::Vector3d& base_pos, 
                                const Eigen::Vector4d& base_quat, 
                                const Eigen::Vector3d& base_linear_vel, 
                                const Eigen::Vector3d& base_angular_vel, 
                                const Eigen::Vector3d& base_linear_acc, 
                                const Eigen::Vector3d& base_angular_acc,
                                const Eigen::VectorXd& qJ, 
                                const Eigen::VectorXd& dqJ,
                                const Eigen::VectorXd& ddqJ) {
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


const Eigen::Vector3d& RobotModel::getBasePosition() const {
  return data_.oMf[imu_frame_].translation();
}


const Eigen::Matrix3d& RobotModel::getBaseRotation() const {
  return data_.oMf[imu_frame_].rotation();
}


const Eigen::Vector3d& RobotModel::getContactPosition(const int contact_id) const {
  assert(contact_id >= 0);
  assert(contact_id < contact_frames_.size());
  return data_.oMf[contact_frames_[contact_id]].translation();
}


const Eigen::Matrix3d& RobotModel::getContactRotation(const int contact_id) const {
  assert(contact_id >= 0);
  assert(contact_id < contact_frames_.size());
  return data_.oMf[contact_frames_[contact_id]].rotation();
}


const Eigen::Block<const Eigen::MatrixXd> RobotModel::getContactJacobian(const int contact_id) const {
  assert(contact_id >= 0);
  assert(contact_id < contact_frames_.size());
  return jac_6d_[contact_id].topRows(3);
}


const Eigen::Block<const Eigen::MatrixXd> RobotModel::getJointContactJacobian(const int contact_id) const {
  assert(contact_id >= 0);
  assert(contact_id < contact_frames_.size());
  return jac_6d_[contact_id].topRightCorner(3, nJ());
}


const Eigen::VectorXd& RobotModel::getInverseDynamics() const {
  return tau_;
}


const Eigen::VectorBlock<const Eigen::VectorXd> RobotModel::getJointInverseDynamics() const {
  return tau_.tail(nJ());
}


const std::vector<int>& RobotModel::getContactFrames() const {
  return contact_frames_;
}


int RobotModel::nq() const {
  return model_.nq;
}


int RobotModel::nv() const {
  return model_.nv;
}


int RobotModel::nJ() const {
  return model_.nv-6;
}


int RobotModel::numContacts() const {
  return contact_frames_.size();
}

} // namespace legged_state_estimator
