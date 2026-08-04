#include <legged_state_estimator/legged_state_estimator.hpp>

#include <stdexcept>
#include <string>


namespace legged_state_estimator 
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedStateEstimator::LeggedStateEstimator(const LeggedStateEstimatorSettings& settings)
    : settings_(settings), inekf_(settings.inekf_noise_params),
      leg_kinematics_(), robot_model_(settings.urdf_path, 
        settings.imu_frame, settings.contact_frames),
      contact_estimator_(robot_model_, settings.contact_estimator_settings),
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
      base_quat_estimate_(Eigen::Quaterniond::Identity().coeffs()) 
  {
    if(settings.sampling_time <= 0.0) 
    {
      throw std::invalid_argument(
        "[LeggedStateEstimator] invalid argment: sampling_time must be positive");
    }

    const scalar_t contact_position_cov = settings.contact_position_noise * settings.contact_position_noise;
    const scalar_t contact_rotation_cov = settings.contact_rotation_noise * settings.contact_rotation_noise;
    
    matrix6_t cov_leg = matrix6_t::Zero();
    cov_leg.topLeftCorner<3, 3>() = contact_position_cov * matrix3_t::Identity();
    cov_leg.bottomRightCorner<3, 3>() = contact_rotation_cov * matrix3_t::Identity();
    for(int  i = 0;  i < settings.contact_frames.size(); ++i) 
    {
      leg_kinematics_.emplace_back(i, Eigen::Matrix4d::Identity(), cov_leg);
    }
    imu_raw_.setZero();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedStateEstimator::LeggedStateEstimator(): settings_(), inekf_(), leg_kinematics_(), 
    robot_model_(), contact_estimator_(), lpf_gyro_accel_world_(), lpf_lin_accel_world_(), 
    lpf_dqJ_(), lpf_ddqJ_(), lpf_tauJ_(), imu_gyro_raw_world_(vector3_t::Zero()),  
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
    base_quat_estimate_(Eigen::Quaterniond::Identity().coeffs()) { }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedStateEstimator::~LeggedStateEstimator() {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimator::init(const vector3_t& base_pos, 
    const vector4_t& base_quat, const vector3_t& base_lin_vel_world, 
    const vector3_t& imu_gyro_bias, const vector3_t& imu_lin_accel_bias) 
  {
    InEKFState initial_state;
    initial_state.setPosition(base_pos);
    initial_state.setRotation(Eigen::Quaterniond(base_quat).toRotationMatrix());
    initial_state.setVelocity(base_lin_vel_world);
    initial_state.setGyroscopeBias(imu_gyro_bias);
    initial_state.setAccelerometerBias(imu_lin_accel_bias);
    inekf_.setState(initial_state);

    lpf_gyro_accel_world_.reset();
    lpf_lin_accel_world_.reset();
    lpf_dqJ_.reset();
    lpf_ddqJ_.reset();
    lpf_tauJ_.reset();
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
    robot_model_.updateKinematics(vector3_t::Zero(), base_quat, qJ);

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

    // Process IMU measurements in InEKF
    imu_raw_.template head<3>() = imu_gyro_raw;
    imu_raw_.template tail<3>() = imu_lin_accel_raw;
    inekf_.Propagate(imu_raw_, settings_.sampling_time);

    // Process IMU measurements in LPFs (linear acceleration)
    imu_lin_accel_raw_world_.noalias() = getBaseRotationEstimate() * (imu_lin_accel_raw - getIMULinearAccelerationBiasEstimate());
    lpf_lin_accel_world_.update(imu_lin_accel_raw_world_);
    imu_lin_accel_local_.noalias() = getBaseRotationEstimate().transpose() * lpf_lin_accel_world_.getEstimate();
    
    // Process IMU measurements in LPFs (angular acceleration via a finite difference)
    if(settings_.dynamic_contact_estimation) 
    {
      imu_gyro_raw_world_.noalias() = getBaseRotationEstimate() * (imu_gyro_raw - getIMUGyroBiasEstimate());
      imu_gyro_accel_world_.noalias() = (imu_gyro_raw_world_ - imu_gyro_raw_world_prev_) / settings_.sampling_time;
      lpf_gyro_accel_world_.update(imu_gyro_accel_world_);
      imu_gyro_accel_local_.noalias() = getBaseRotationEstimate().transpose() * lpf_gyro_accel_world_.getEstimate();
      imu_gyro_raw_world_prev_ = imu_gyro_raw_world_;
    }

    // Process joint measurements in LPFs 
    if(settings_.dynamic_contact_estimation) 
    {

      lpf_ddqJ_.update((dqJ-lpf_dqJ_.getEstimate())/settings_.sampling_time);
    }
    lpf_dqJ_.update(dqJ);
    lpf_tauJ_.update(tauJ);

    // Update contact info
    robot_model_.updateLegKinematics(qJ);
    if(settings_.dynamic_contact_estimation) 
    {
      robot_model_.updateDynamics(getBasePositionEstimate(), getBaseQuaternionEstimate(),
        getBaseLinearVelocityEstimateLocal(), imu_gyro_raw, imu_lin_accel_local_, 
        imu_gyro_accel_local_, qJ, dqJ, lpf_ddqJ_.getEstimate());
    }
    else 
    {
      robot_model_.updateLegDynamics(qJ, dqJ);
    }

    contact_estimator_.update(robot_model_, lpf_tauJ_.getEstimate());
    inekf_.setContacts(contact_estimator_.getContactState());

    for(int  i = 0;  i < robot_model_.numContacts(); ++i) 
    {
      leg_kinematics_[i].setContactPosition(
        robot_model_.getContactPosition(i)-robot_model_.getBasePosition());
      
      const scalar_t contact_force_cov = contact_estimator_.getContactForceCovariance()[i];
      
      leg_kinematics_[i].setContactPositionCovariance(
        contact_force_cov*matrix3_t::Identity());
    }

    // Process kinematics measurements in InEKF
    inekf_.CorrectKinematics(leg_kinematics_);

    // Restore estimates
    base_pos_estimate_ = inekf_.getState().getPosition();
    base_rot_estimate_ = inekf_.getState().getRotation();
    base_quat_estimate_ = Eigen::Quaterniond(inekf_.getState().getRotation()).coeffs();
    base_lin_vel_world_estimate_ = inekf_.getState().getVelocity();
    base_lin_vel_local_estimate_ = inekf_.getState().getRotation().transpose() * base_lin_vel_world_estimate_;
    base_ang_vel_world_estimate_ = imu_gyro_raw_world_;
    base_ang_vel_local_estimate_ = imu_gyro_raw - getIMUGyroBiasEstimate();
    imu_gyro_bias_estimate_ = inekf_.getState().getGyroscopeBias();
    imu_lin_acc_bias_estimate_ = inekf_.getState().getAccelerometerBias();
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
    return contact_estimator_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const LeggedStateEstimatorSettings& LeggedStateEstimator::getSettings() const 
  {
    return settings_;
  }
} // namespace legged_state_estimator
