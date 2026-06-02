#ifndef LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>


namespace legged_state_estimator {

///
/// @class LowPassFilter
/// @brief A simple low pass filter.
///
template <typename Scalar, int dim=Eigen::Dynamic>
class LowPassFilter {
public:
  using Vector = Eigen::Matrix<Scalar, dim, 1>;

  ///
  /// @brief Constructs a low pass filter.
  /// @param[in] sampling_time Sampling time.
  /// @param[in] cutoff_frequency The cut-off frequency.
  /// @param[in] dynamic_size Size parameter used when the template paramter dim
  /// is set to Eigen::Dynamic.
  ///
  LowPassFilter(const Scalar sampling_time, const Scalar cutoff_frequency,
                const int dynamic_size=0)
    : estimate_(),
      alpha_(0.0) {
    if (sampling_time <= 0) {
      throw std::invalid_argument(
          "[LowPassFilter] invalid argment: sampling_time must be positive");
    }
    if (cutoff_frequency <= 0) {
      throw std::invalid_argument(
          "[LowPassFilter] invalid argment: cutoff_frequency must be positive");
    }
    if (dim == Eigen::Dynamic && dynamic_size <= 0) {
      throw std::invalid_argument(
          "[LowPassFilter] invalid argment: dynamic_size must be positive");
    }
    const Scalar tau = 1.0 / (2.0*M_PI*cutoff_frequency);
    alpha_ = tau / (tau + sampling_time);
    if (dim == Eigen::Dynamic) {
      estimate_.resize(dynamic_size);
    }
    estimate_.setZero();
  }

  ///
  /// @brief Default constructor. 
  ///
  LowPassFilter()
    : estimate_(),
      alpha_(0.0) {
  }

  ///
  /// @brief Default destructor. 
  ///
  ~LowPassFilter() = default;

  LowPassFilter(const LowPassFilter&) = default;
  LowPassFilter& operator=(const LowPassFilter&) = default;
  LowPassFilter(LowPassFilter&&) noexcept = default;
  LowPassFilter& operator=(LowPassFilter&&) noexcept = default;

  ///
  /// @brief Reset the filter and reset the estimate to zero. 
  ///
  void reset() {
    estimate_.setZero();
  }

  ///
  /// @brief Reset the filter with input estimate. 
  /// @param[in] estimate An initial estimate. 
  ///
  void reset(const Vector& estimate) {
    estimate_ = estimate;
  }

  ///
  /// @brief Updates the estimate.
  /// @param[in] observation Observation. 
  ///
  void update(const Vector& observation) {
    estimate_.array() *= alpha_;
    estimate_.noalias() += (1.0-alpha_) * observation;
  }

  ///
  /// @brief Gets the estimate.
  /// @return const reference to the estimate.
  ///
  const Vector& getEstimate() const {
    return estimate_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector estimate_;
  Scalar alpha_;
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_ 