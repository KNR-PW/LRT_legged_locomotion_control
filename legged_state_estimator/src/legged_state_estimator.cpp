#include <legged_state_estimator/legged_state_estimator.hpp>

#include <stdexcept>
#include <string>


namespace legged_state_estimator 
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedStateEstimator::LeggedStateEstimator(const std::string& urdf_path,
    const LeggedStateEstimatorSettings& settings):
      settings_(settings), inekf_(settings.inekf_noise_params),
      leg_kinematics_(), robot_model_(urdf_path, settings.base_frame,
        settings.imu_frame, settings.contact_frames),
      lpf_gyro_accel_world_(settings.sampling_time, settings.lpf_gyro_accel_cutoff_frequency),
      lpf_lin_accel_world_(settings.sampling_time, settings.lpf_lin_accel_cutoff_frequency),
      lpf_dqJ_(settings.sampling_time, settings.lpf_dqJ_cutoff_frequency, robot_model_.nJ()),
      lpf_ddqJ_(settings.sampling_time, settings.lpf_ddqJ_cutoff_frequency, robot_model_.nJ()),
      lpf_tauJ_(settings.sampling_time, settings.lpf_tauJ_cutoff_frequency, robot_model_.nJ()),
      imu_gyro_raw_world_(vector3_t::Zero()), 
      imu_gyro_raw_world_prev_(vector3_t::Zero()), 
      imu_gyro_accel_world_(vector3_t::Zero()), 
      imu_gyro_accel_local_(vector3_t::Zero()), 
      imu_lin_accel_raw_world_(vector3_t::Zero()), 
      imu_lin_accel_local_(vector3_t::Zero()),
      base_pos_estimate_(vector3_t::Zero()),
      base_lin_vel_world_estimate_(vector3_t::Zero()), 
      base_lin_vel_local_estimate_(vector3_t::Zero()),
      base_ang_vel_world_estimate_(vector3_t::Zero()), 
      base_ang_vel_local_estimate_(vector3_t::Zero()),
      imu_gyro_bias_estimate_(vector3_t::Zero()), 
      imu_lin_acc_bias_estimate_(vector3_t::Zero()),
      base_rot_estimate_(matrix3_t::Identity()),
      imu_raw_(vector6_t::Zero()),
      base_quat_estimate_(Eigen::Quaterniond::Identity().coeffs()),
      filters_initialized_(false)
  {
    if(settings.sampling_time <= 0.0) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: sampling_time must be positive");
    }

    if(settings.use_contact_estimator)
    {
      contact_estimator_ptr_ = std::make_unique<ContactEstimator>(robot_model_, 
        settings.contact_estimator_settings);
    }

    const scalar_t contact_position_cov = settings.contact_position_noise * settings.contact_position_noise;
    const scalar_t contact_rotation_cov = settings.contact_rotation_noise * settings.contact_rotation_noise;
    
    // InEKF covariance convention: rotation first, then position
    matrix6_t cov_leg = matrix6_t::Zero();
    cov_leg.topLeftCorner<3, 3>() = contact_rotation_cov * matrix3_t::Identity();
    cov_leg.bottomRightCorner<3, 3>() = contact_position_cov * matrix3_t::Identity();
    for(int  i = 0;  i < settings.contact_frames.size(); ++i) 
    {
      leg_kinematics_.emplace_back(i, Eigen::Matrix4d::Identity(), cov_leg);
    }
    imu_raw_.setZero();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::init(const vector3_t& base_pos, 
    const vector4_t& base_quat, const vector3_t& base_lin_vel_world, 
    const vector3_t& imu_gyro_bias, const vector3_t& imu_lin_accel_bias) 
  {
    // InEKF estimates the IMU state, IMU is rigidly attached to the base
    // (angular velocity is assumed to be zero at initialization)
    const pinocchio::SE3& imu_in_base = robot_model_.getImuPlacementInBase();
    const matrix3_t base_rotation = Eigen::Quaterniond(base_quat).toRotationMatrix();
    InEKFState initial_state;
    initial_state.setPosition(base_pos + base_rotation * imu_in_base.translation());
    initial_state.setRotation(base_rotation * imu_in_base.rotation());
    initial_state.setVelocity(base_lin_vel_world);
    initial_state.setGyroscopeBias(imu_gyro_bias);
    initial_state.setAccelerometerBias(imu_lin_accel_bias);
    inekf_.setState(initial_state);

    lpf_gyro_accel_world_.reset();
    lpf_lin_accel_world_.reset();
    lpf_dqJ_.reset();
    lpf_ddqJ_.reset();
    lpf_tauJ_.reset();
    // Filters are started from the first measurements in the next update
    filters_initialized_ = false;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::init(const vector3_t& base_pos,
    const vector4_t& base_quat, const vector_t& qJ, 
    const std::vector<scalar_t>& ground_height, 
    const vector3_t& base_lin_vel_world, const vector3_t& imu_gyro_bias,
    const vector3_t& imu_lin_accel_bias) 
  {
    if(qJ.size() != robot_model_.nJ()) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: qJ.size() must be " + std::to_string(robot_model_.nJ()));
    }
    if(ground_height.size() != robot_model_.numContacts()) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: ground_height.size() must be " + std::to_string(robot_model_.numContacts()));
    }
    // Kinematics is computed for the floating base (root joint), base frame is rigidly attached to it
    const matrix3_t base_rotation = Eigen::Quaterniond(base_quat).toRotationMatrix();
    const matrix3_t root_rotation = base_rotation * robot_model_.getBasePlacementInRoot().rotation().transpose();
    robot_model_.updateKinematics(vector3_t::Zero(), Eigen::Quaterniond(root_rotation).coeffs(), qJ);

    scalar_t base_height = 0;
    for(int  i = 0;  i < robot_model_.numContacts(); ++i) 
    {
      base_height += (robot_model_.getBasePosition().coeff(2)
        - robot_model_.getContactPosition(i).coeff(2) + ground_height[i]);
    }

    base_height /= robot_model_.numContacts();

    const vector3_t base_pos_correct = {base_pos.coeff(0), base_pos.coeff(1), 
      base_height};

    init(base_pos_correct, base_quat, base_lin_vel_world, 
      imu_gyro_bias, imu_lin_accel_bias);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::update(const vector3_t& imu_gyro_raw, 
    const vector3_t& imu_lin_accel_raw, const vector_t& qJ, 
    const vector_t& dqJ, const vector_t& tauJ) 
  {
    update(imu_gyro_raw, imu_lin_accel_raw, qJ, dqJ, tauJ, settings_.sampling_time);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::update(const vector3_t& imu_gyro_raw, 
    const vector3_t& imu_lin_accel_raw, const vector_t& qJ, 
    const vector_t& dqJ, const vector_t& tauJ, const scalar_t dt) 
  {
    if(!settings_.use_contact_estimator)
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] wrong version of update() method used, use_contact_estimator is set to false");
    }

    checkUpdateArguments(qJ, dqJ, tauJ, dt);

    processMeasurements(imu_gyro_raw, imu_lin_accel_raw, qJ, dqJ, tauJ, dt);

    if(contact_estimator_ptr_)
    {
      contact_estimator_ptr_->update(robot_model_, lpf_tauJ_.getEstimate());
      inekf_.setContacts(contact_estimator_ptr_->getContactState());
    }

    correctAndRestoreEstimates(imu_gyro_raw);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::update(const vector3_t& imu_gyro_raw, 
    const vector3_t& imu_lin_accel_raw, 
    const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ, const ocs2::vector_t& tauJ, 
    const std::vector<std::pair<int,bool>>& contacts)
  {
    update(imu_gyro_raw, imu_lin_accel_raw, qJ, dqJ, tauJ, contacts, settings_.sampling_time);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::update(const vector3_t& imu_gyro_raw, 
    const vector3_t& imu_lin_accel_raw, 
    const ocs2::vector_t& qJ, const ocs2::vector_t& dqJ, const ocs2::vector_t& tauJ, 
    const std::vector<std::pair<int,bool>>& contacts, const scalar_t dt)
  {
    if(settings_.use_contact_estimator)
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] wrong version of update() method used, use_contact_estimator is set to true");
    }

    checkUpdateArguments(qJ, dqJ, tauJ, dt);

    processMeasurements(imu_gyro_raw, imu_lin_accel_raw, qJ, dqJ, tauJ, dt);

    inekf_.setContacts(contacts);

    correctAndRestoreEstimates(imu_gyro_raw);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::checkUpdateArguments(const vector_t& qJ, 
    const vector_t& dqJ, const vector_t& tauJ, const scalar_t dt) const
  {
    if(qJ.size() != robot_model_.nJ()) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: qJ.size() must be " + std::to_string(robot_model_.nJ()));
    }

    if(dqJ.size() != robot_model_.nJ()) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: dqJ.size() must be " + std::to_string(robot_model_.nJ()));
    }

    if(tauJ.size() != robot_model_.nJ()) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: tauJ.size() must be " + std::to_string(robot_model_.nJ()));
    }

    if(!(dt > 0.0)) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: dt must be positive, got " + std::to_string(dt));
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::processMeasurements(const vector3_t& imu_gyro_raw, 
    const vector3_t& imu_lin_accel_raw, const vector_t& qJ, 
    const vector_t& dqJ, const vector_t& tauJ, const scalar_t dt)
  {
    // Process IMU measurements in InEKF
    imu_raw_.template head<3>() = imu_gyro_raw;
    imu_raw_.template tail<3>() = imu_lin_accel_raw;
    inekf_.Propagate(imu_raw_, dt);

    // Bias corrected IMU measurements expressed with the propagated (current) state
    const InEKFState& state = inekf_.getState();
    const matrix3_t rotation = state.getRotation();
    const vector3_t imu_gyro_local = imu_gyro_raw - state.getGyroscopeBias();
    imu_gyro_raw_world_.noalias() = rotation * imu_gyro_local;
    // Accelerometer measures specific force, gravity is added to get the acceleration
    imu_lin_accel_raw_world_.noalias() = rotation * (imu_lin_accel_raw - state.getAccelerometerBias());
    imu_lin_accel_raw_world_.coeffRef(2) += MINUS_GRAVITY_VALUE;

    // Start filters from the first measurements instead of zero
    if(!filters_initialized_)
    {
      lpf_lin_accel_world_.reset(imu_lin_accel_raw_world_);
      lpf_gyro_accel_world_.reset();
      imu_gyro_raw_world_prev_ = imu_gyro_raw_world_;
      lpf_dqJ_.reset(dqJ);
      lpf_ddqJ_.reset();
      lpf_tauJ_.reset(tauJ);
      filters_initialized_ = true;
    }

    // Process IMU measurements in LPFs (linear acceleration)
    lpf_lin_accel_world_.update(imu_lin_accel_raw_world_, dt);
    imu_lin_accel_local_.noalias() = rotation.transpose() * lpf_lin_accel_world_.getEstimate();

    // Process IMU measurements in LPFs (angular acceleration via a finite difference)
    if(settings_.dynamic_contact_estimation)
    {
      imu_gyro_accel_world_.noalias() = (imu_gyro_raw_world_ - imu_gyro_raw_world_prev_) / dt;
      lpf_gyro_accel_world_.update(imu_gyro_accel_world_, dt);
      imu_gyro_accel_local_.noalias() = rotation.transpose() * lpf_gyro_accel_world_.getEstimate();
    }
    imu_gyro_raw_world_prev_ = imu_gyro_raw_world_;

    // Process joint measurements in LPFs
    if(settings_.dynamic_contact_estimation)
    {
      lpf_ddqJ_.update((dqJ - lpf_dqJ_.getEstimate()) / dt, dt);
    }
    lpf_dqJ_.update(dqJ, dt);
    lpf_tauJ_.update(tauJ, dt);

    // Update contact info (dynamics and contact Jacobians are used only by the contact estimator)
    if(!contact_estimator_ptr_)
    {
      robot_model_.updateLegKinematics(qJ, pinocchio::LOCAL_WORLD_ALIGNED, false);
      return;
    }

    // Kinematics and dynamics use the estimated floating base orientation: gravity points in world -z
    // and contact Jacobians (so also contact forces) are expressed in the world frame
    const pinocchio::SE3& imu_in_root = robot_model_.getImuPlacementInRoot();
    const matrix3_t root_rotation = rotation * imu_in_root.rotation().transpose();
    const vector4_t root_quat = Eigen::Quaterniond(root_rotation).coeffs();
    robot_model_.updateKinematics(vector3_t::Zero(), root_quat, qJ);
    if(settings_.dynamic_contact_estimation)
    {
      // Floating base (root joint) state from the IMU state, IMU is rigidly attached at root_to_imu
      const vector3_t root_to_imu = root_rotation * imu_in_root.translation();
      const vector3_t& ang_vel_world = imu_gyro_raw_world_;
      const vector3_t& ang_accel_world = lpf_gyro_accel_world_.getEstimate();
      const vector3_t root_lin_vel_world = state.getVelocity() - ang_vel_world.cross(root_to_imu);
      const vector3_t root_lin_accel_world = lpf_lin_accel_world_.getEstimate() - ang_accel_world.cross(root_to_imu)
        - ang_vel_world.cross(ang_vel_world.cross(root_to_imu));
      const vector3_t root_ang_vel_local = imu_in_root.rotation() * imu_gyro_local;
      const vector3_t root_ang_accel_local = imu_in_root.rotation() * imu_gyro_accel_local_;
      const vector3_t root_lin_vel_local = root_rotation.transpose() * root_lin_vel_world;
      // Pinocchio's free-flyer acceleration is the derivative of the local velocity: R^T*a_world - w x v_local
      const vector3_t root_lin_accel_local = root_rotation.transpose() * root_lin_accel_world
        - root_ang_vel_local.cross(root_lin_vel_local);
      robot_model_.updateDynamics(state.getPosition() - root_to_imu, root_quat,
        root_lin_vel_local, root_ang_vel_local, root_lin_accel_local,
        root_ang_accel_local, qJ, dqJ, lpf_ddqJ_.getEstimate());
    }
    else
    {
      robot_model_.updateLegDynamics(qJ, dqJ, root_quat);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::correctAndRestoreEstimates(const vector3_t& imu_gyro_raw)
  {
    // Contact positions must be expressed in the IMU frame (frame of the InEKF state)
    const matrix3_t imu_rotation_transposed = robot_model_.getImuRotation().transpose();
    const scalar_t contact_position_cov = settings_.contact_position_noise * settings_.contact_position_noise;
    for(int  i = 0;  i < robot_model_.numContacts(); ++i)
    {
      leg_kinematics_[i].setContactPosition(imu_rotation_transposed
        * (robot_model_.getContactPosition(i) - robot_model_.getImuPosition()));

      scalar_t contact_cov = contact_position_cov;
      if(contact_estimator_ptr_)
      {
        contact_cov += contact_estimator_ptr_->getContactForceCovariance()[i];
      }

      leg_kinematics_[i].setContactPositionCovariance(
        contact_cov*matrix3_t::Identity());
    }

    // Process kinematics measurements in InEKF
    inekf_.CorrectKinematics(leg_kinematics_);

    // Restore estimates, InEKF estimates the IMU state which is converted to the base frame
    const InEKFState& state = inekf_.getState();
    const pinocchio::SE3& imu_in_base = robot_model_.getImuPlacementInBase();
    imu_gyro_bias_estimate_ = state.getGyroscopeBias();
    imu_lin_acc_bias_estimate_ = state.getAccelerometerBias();
    const vector3_t imu_ang_vel_local = imu_gyro_raw - imu_gyro_bias_estimate_;

    base_rot_estimate_.noalias() = state.getRotation() * imu_in_base.rotation().transpose();
    base_quat_estimate_ = Eigen::Quaterniond(base_rot_estimate_).coeffs();
    const vector3_t base_to_imu = base_rot_estimate_ * imu_in_base.translation();
    base_pos_estimate_ = state.getPosition() - base_to_imu;
    base_ang_vel_local_estimate_.noalias() = imu_in_base.rotation() * imu_ang_vel_local;
    base_ang_vel_world_estimate_.noalias() = base_rot_estimate_ * base_ang_vel_local_estimate_;
    base_lin_vel_world_estimate_ = state.getVelocity() - base_ang_vel_world_estimate_.cross(base_to_imu);
    base_lin_vel_local_estimate_.noalias() = base_rot_estimate_.transpose() * base_lin_vel_world_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::setGroundNormals(const std::vector<vector3_t>& ground_normals)
  {
    if(!ground_normals.empty() && ground_normals.size() != robot_model_.numContacts())
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: ground_normals.size() must be 0 (flat ground) or "
        + std::to_string(robot_model_.numContacts()));
    }

    std::vector<vector3_t> normalized_ground_normals;
    for(int  i = 0;  i < ground_normals.size(); ++i)
    {
      const scalar_t norm = ground_normals[i].norm();
      if(!(norm > SCALAR_EPSILON))
      {
        throw std::invalid_argument(
          "[LeggedStateEstimator] invalid argment: ground normal " + std::to_string(i) + " must be non-zero");
      }
      normalized_ground_normals.push_back(ground_normals[i] / norm);
    }
    ground_normals_ = std::move(normalized_ground_normals);

    if(contact_estimator_ptr_)
    {
      std::vector<vector3_t> contact_normals(robot_model_.numContacts());
      for(int  i = 0;  i < robot_model_.numContacts(); ++i)
      {
        // Ground normals are normalized above, no ground normals means flat ground
        const vector3_t normal = [&](){
        if(ground_normals_.size() == 0)
        {
          return vector3_t(0.0, 0.0, 1.0);
        }
        else
        {
          return ground_normals_[i];
        }}();
        contact_normals[i] = normal;
      }
      contact_estimator_ptr_->setContactSurfaceNormal(contact_normals);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const std::vector<vector3_t>& LeggedStateEstimator::getGroundNormals() const
  {
    return ground_normals_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& LeggedStateEstimator::getBasePositionEstimate() const
  {
    return base_pos_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const matrix3_t& LeggedStateEstimator::getBaseRotationEstimate() const 
  {
    return base_rot_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector4_t& LeggedStateEstimator::getBaseQuaternionEstimate() const 
  {
    return base_quat_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& LeggedStateEstimator::getBaseLinearVelocityEstimateWorld() const 
  {
    return base_lin_vel_world_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t LeggedStateEstimator::getBaseLinearVelocityEstimateLocal() const 
  {
    return base_lin_vel_local_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& LeggedStateEstimator::getBaseAngularVelocityEstimateWorld() const 
  {
    return base_ang_vel_world_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& LeggedStateEstimator::getBaseAngularVelocityEstimateLocal() const 
  {
    return base_ang_vel_local_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& LeggedStateEstimator::getIMUGyroBiasEstimate() const 
  {
    return imu_gyro_bias_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& LeggedStateEstimator::getIMULinearAccelerationBiasEstimate() const 
  {
    return imu_lin_acc_bias_estimate_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector_t& LeggedStateEstimator::getJointVelocityEstimate() const 
  {
    return lpf_dqJ_.getEstimate();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector_t& LeggedStateEstimator::getJointAccelerationEstimate() const 
  {
    return lpf_ddqJ_.getEstimate();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector_t& LeggedStateEstimator::getJointTorqueEstimate() const 
  {
    return lpf_tauJ_.getEstimate();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const ContactEstimator& LeggedStateEstimator::getContactEstimator() const
  {
    if(!contact_estimator_ptr_)
    {
      throw std::logic_error(
        "[LeggedStateEstimator] contact estimator is not used, use_contact_estimator is set to false");
    }
    return *contact_estimator_ptr_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const LeggedStateEstimatorSettings& LeggedStateEstimator::getSettings() const 
  {
    return settings_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const RobotModel& LeggedStateEstimator::getRobotModel() const 
  {
    return robot_model_;
  }
} // namespace legged_state_estimator
