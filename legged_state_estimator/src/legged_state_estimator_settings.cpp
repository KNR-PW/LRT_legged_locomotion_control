#include <legged_state_estimator/legged_state_estimator_settings.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace legged_state_estimator 
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedStateEstimatorSettings loadLeggedStateEstimatorSettings(const std::string& filename,
    const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged State Estimator Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    LeggedStateEstimatorSettings settings;

    loadData::loadPtreeValue(pt, settings.world_frame, 
      fieldName + ".worldFrameName", verbose);

    loadData::loadPtreeValue(pt, settings.base_frame, 
      fieldName + ".baseFrameName", verbose);

    loadData::loadPtreeValue(pt, settings.imu_frame, 
      fieldName + ".imuFrameName", verbose);

    loadData::loadStdVector(filename, fieldName + ".contactFrames", 
      settings.contact_frames, verbose);

    loadData::loadPtreeValue(pt, settings.use_contact_estimator, 
      fieldName + ".useContactEstimator", verbose);

    if(settings.use_contact_estimator)
    {
      const size_t contact_frames_size = settings.contact_frames.size();

      loadData::loadStdVector(filename, fieldName + ".contactEstimatorSettings.beta0", 
        settings.contact_estimator_settings.beta0, verbose);
      const size_t beta0_size = settings.contact_estimator_settings.beta0.size();
      if(beta0_size != contact_frames_size)
      {
        std::string message = "[LeggedStateEstimator] beta0 size is " +  beta0_size;
        message += ", but should be ";
        message += contact_frames_size;
        throw std::invalid_argument(message);
      }

      loadData::loadStdVector(filename, fieldName + ".contactEstimatorSettings.beta1", 
        settings.contact_estimator_settings.beta1, verbose);
      const size_t beta1_size = settings.contact_estimator_settings.beta1.size();
      if(beta1_size != contact_frames_size)
      {
        std::string message = "[LeggedStateEstimator] beta1 size is " +  beta1_size;
        message += ", but should be ";
        message += contact_frames_size;
        throw std::invalid_argument(message);
      }

      loadData::loadPtreeValue(pt, 
        settings.contact_estimator_settings.contact_force_covariance_alpha, 
        fieldName + ".contactEstimatorSettings.contactForceCovarianceAlpha", verbose);

      loadData::loadPtreeValue(pt, 
        settings.contact_estimator_settings.contact_probability_threshold, 
        fieldName + ".contactEstimatorSettings.contactProbabilityThreshold", verbose);
    }

    scalar_t gyroscope_noise;
    scalar_t gyroscope_bias_noise;

    scalar_t acceleration_noise;
    scalar_t acceleration_bias_noise;

    scalar_t contact_noise;

    loadData::loadPtreeValue(pt, gyroscope_noise, 
      fieldName + ".noiseSettings.gyroscopeNoise", verbose);
    loadData::loadPtreeValue(pt, gyroscope_bias_noise, 
      fieldName + ".noiseSettings.gyroscopeBiasNoise", verbose);
    loadData::loadPtreeValue(pt, acceleration_noise, 
      fieldName + ".noiseSettings.accelerometerNoise", verbose);
    loadData::loadPtreeValue(pt, acceleration_bias_noise, 
      fieldName + ".noiseSettings.accelerometerBiasNoise", verbose);
    loadData::loadPtreeValue(pt, contact_noise, 
      fieldName + ".noiseSettings.contactNoise", verbose);

    settings.inekf_noise_params.setGyroscopeNoise(gyroscope_noise);
    settings.inekf_noise_params.setGyroscopeBiasNoise(gyroscope_bias_noise);
    settings.inekf_noise_params.setAccelerometerNoise(acceleration_noise);
    settings.inekf_noise_params.setAccelerometerBiasNoise(acceleration_bias_noise);
    settings.inekf_noise_params.setContactNoise(contact_noise);

    loadData::loadPtreeValue(pt, settings.dynamic_contact_estimation, 
      fieldName + ".dynamicContactEstimation", verbose);

    loadData::loadPtreeValue(pt, settings.contact_position_noise, 
      fieldName + ".contactPositionNoise", verbose);

    loadData::loadPtreeValue(pt, settings.contact_rotation_noise, 
      fieldName + ".contactRotationNoise", verbose);

    loadData::loadPtreeValue(pt, settings.sampling_time, 
      fieldName + ".samplingTime", verbose);

    loadData::loadPtreeValue(pt, settings.lpf_gyro_cutoff_frequency, 
      fieldName + ".lpfGyroCutoffFrequency", verbose);

    loadData::loadPtreeValue(pt, settings.lpf_gyro_accel_cutoff_frequency, 
      fieldName + ".lpfGyroAccelCutoffFrequency", verbose);

    loadData::loadPtreeValue(pt, settings.lpf_lin_accel_cutoff_frequency, 
      fieldName + ".lpfLinAccelCutoffFrequency", verbose);

    loadData::loadPtreeValue(pt, settings.lpf_dqJ_cutoff_frequency, 
      fieldName + ".lpfDqJCutoffFrequency", verbose);

    loadData::loadPtreeValue(pt, settings.lpf_ddqJ_cutoff_frequency, 
      fieldName + ".lpfDdqJCutoffFrequency", verbose);

    loadData::loadPtreeValue(pt, settings.lpf_tauJ_cutoff_frequency, 
      fieldName + ".lpfTauJCutoffFrequency", verbose);

    return settings;
  }
} // namespace legged_state_estimator