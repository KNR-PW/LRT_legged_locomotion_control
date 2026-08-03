#ifndef __LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR___
#define __LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR___

#include <vector>
#include <utility>
#include <iostream>

#include <Eigen/Core>
#include "Eigen/StdVector"

#include "legged_state_estimator/robot_model.hpp"


namespace legged_state_estimator {

///
/// @class ContactEstimatorSettings
/// @brief Settings of the contact estimator.
///
struct ContactEstimatorSettings {
  ///
  /// @brief A parameter of the logistic regression for the contact state (on/off).
  ///
  std::vector<ocs2::scalar_t> beta0;

  ///
  /// @brief A parameter of the logistic regression for the contact state (on/off).
  ///
  std::vector<ocs2::scalar_t> beta1;

  ///
  /// @brief A parameter in computing the contact force covariances.
  ///
  ocs2::scalar_t contact_force_covariance_alpha;

  ///
  /// @brief Threshold to determine the contact state from contact probabilities.
  ///
  ocs2::scalar_t contact_probability_threshold;
};


///
/// @class ContactEstimator
/// @brief Contact estimator.
///
class ContactEstimator {
public:
  ///
  /// @brief Constructor.
  /// @param[in] robot_model Robot model.
  /// @param[in] settings Contact estimator settings.
  ///
  ContactEstimator(const RobotModel& robot_model, 
                   const ContactEstimatorSettings& settings);

  ///
  /// @brief Default constructor.
  ///
  ContactEstimator();

  ///
  /// @brief Default destructor.
  ///
  ~ContactEstimator() = default;

  ContactEstimator(const ContactEstimator&) = default;
  ContactEstimator& operator=(const ContactEstimator&) = default;
  ContactEstimator(ContactEstimator&&) noexcept = default;
  ContactEstimator& operator=(ContactEstimator&&) noexcept = default;

  ///
  /// @brief Resets the estimation.
  ///
  void reset();

  ///
  /// @brief Updates the estimation.
  /// @param[in] robot_model Robot model.
  /// @param[in] tauJ Current joint torque.
  ///
  void update(const RobotModel& robot_model, const ocs2::vector_t& tauJ);

  ///
  /// @brief Set the parameters.
  /// @param[in] settings Settings.
  ///
  void setParameters(const ContactEstimatorSettings& settings);

  ///
  /// @brief Get the contact state, i.e., on/off of the contacts.
  ///
  const std::vector<std::pair<int, bool>>& getContactState() const;

  ///
  /// @brief Get the contact probabilities.
  /// @return const reference to the contact probabilities.
  ///
  const std::vector<ocs2::scalar_t>& getContactProbability() const;

  ///
  /// @brief Get the contact force covariances.
  /// @return const reference to the contact force covariances.
  ///
  const std::vector<ocs2::scalar_t>& getContactForceCovariance() const;

  ///
  /// @brief Get the contact force estimates.
  /// @return const reference to the contact force estimates.
  ///
  const std::vector<ocs2::vector3_t>& getContactForceEstimate() const;

  ///
  /// @brief Get the normal contact force estimates.
  /// @return const reference to the normal contact force estimates.
  ///
  const std::vector<ocs2::scalar_t>& getNormalContactForceEstimate() const;

  ///
  /// @brief Get the contact normal surface.
  /// @return const reference to the contact normal surface.
  ///
  const std::vector<ocs2::vector3_t>& getContactSurfaceNormal() const;

  ///
  /// @brief Sets the contact normal surface.
  /// @param[in] Contact Contact normal vectors.
  ///
  void setContactSurfaceNormal(const std::vector<ocs2::vector3_t>& contact_surface_normal);

  void disp(std::ostream& os) const;

  friend std::ostream& operator<<(std::ostream& os, const ContactEstimator& d);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ContactEstimatorSettings settings_;
  std::vector<ocs2::vector3_t> contact_force_estimate_, 
                               contact_force_estimate_prev_, 
                               contact_surface_normal_;
  std::vector<ocs2::scalar_t> normal_contact_force_estimate_, 
                      normal_contact_force_estimate_prev_, 
                      contact_probability_, contact_force_covariance_;
  std::vector<std::pair<int, bool>> contact_state_;
  int num_contacts_;
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR___ 