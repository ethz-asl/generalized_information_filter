/*
 * filter.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include <algorithm>  // Required for std::find

#include "filter_test/estimator.h"
#include "filter_test/utils/logging.h"

namespace tsif {

bool Estimator::defineState(std::vector<BlockTypeId> state_types) {
  state_types_ = state_types;
  state_.defineState(state_types);
  return true;
}

bool Estimator::addResidual(
    ResidualBase* residual, const std::vector<size_t>& first_keys,
    const std::vector<size_t>& second_keys,
    const std::vector<size_t>& measurement_keys) {
  const bool kIsPredictionResidual = false;
  return addResidualImplementation(
      residual, first_keys, second_keys, measurement_keys,
      kIsPredictionResidual);
}

bool Estimator::addPredictionResidual(
    ResidualBase* residual, const std::vector<size_t>& first_keys,
    const std::vector<size_t>& second_keys,
    const std::vector<size_t>& measurement_keys) {
  const bool kIsPredictionResidual = true;
  return addResidualImplementation(
      residual, first_keys, second_keys, measurement_keys,
      kIsPredictionResidual);
}

bool Estimator::addResidualImplementation(
    ResidualBase* residual, const std::vector<size_t>& first_keys,
    const std::vector<size_t>& second_keys,
    const std::vector<size_t>& measurement_keys, const bool use_for_prediction) {
  CHECK_NOTNULL(residual);
  CHECK(state_.dimension_ > 0);  // Check if state is defined.
  CHECK(!is_initialized_) << "Extending the state or adding residuals after "
                             "initialization is not suported yet";

  measurement_manager_.prepareTimelines(
      measurement_keys, residual->is_mergeable_);

  return problem_builder_.addResidual(
      residual, first_keys, second_keys, measurement_keys, use_for_prediction);
}

void Estimator::addMeasurement(
    size_t timeline_key, int64_t timestamp_ns, MeasurementBase* measurement) {
  CHECK_NOTNULL(measurement);
  if (timestamp_previous_update_ns_ > timestamp_ns) {
    std::cout
        << "timestamp of measurement is older than current estimator time."
        << " State buffer not yet implemented, dropping measurement: "
        << measurement->getMeasurementName() << " at time "
        << std::to_string(timestamp_ns) << std::endl;
    return;
  }

  TSIF_LOG(
      "added " << measurement->getMeasurementName() << " at time "
               << std::to_string(timestamp_ns));
  measurement_manager_.addMeasurement(timeline_key, timestamp_ns, measurement);
  runEstimator();
}

void Estimator::runEstimator() {
  MeasurementBuffer measurement_buffer;
  while (measurement_manager_.updateStrategy(
      timestamp_previous_update_ns_, &measurement_buffer)) {
    if (!is_initialized_) {
      // This is the first run and we have to initialize everything
      TSIF_LOG(
          "Init filter at time "
          << std::to_string(measurement_buffer.timestamp_ns));
      init(measurement_buffer);
      is_initialized_ = true;
      timestamp_previous_update_ns_ = measurement_buffer.timestamp_ns;
      return;
    }

    TSIF_LOG(
        "predictAndUpdate at time "
        << std::to_string(measurement_buffer.timestamp_ns));
    FilterProblemDescription problem_description =
        problem_builder_.getFilterProblemDescription(measurement_buffer);
    filter_.predictAndUpdate(
        measurement_buffer, problem_description, state_, information_, &state_,
        &information_);
    timestamp_previous_update_ns_ = measurement_buffer.timestamp_ns;
  }
}

void Estimator::initStateValue(const size_t key, const VectorXRef& value) {
  state_.setBlock(key, value);
}

void Estimator::printState() const {
  std::cout << state_.print() << std::endl;
}

void Estimator::printTimeline() const {
  measurement_manager_.printTimeline();
}

void Estimator::printResiduals() const {
  problem_builder_.printResiduals(state_);
}

void Estimator::checkResiduals() const {
  State random_state = state_;
  random_state.setRandom();
  problem_builder_.checkResiduals(random_state);
}

bool Estimator::init(const MeasurementBuffer& measurement_buffer) {
  information_.resize(state_.minimal_dimension_, state_.minimal_dimension_);
  information_.setZero();
  filter_.init(state_, problem_builder_.getTotalResidualDimension());
  // Call custom initialization to initialize state and information matrix.
  state_initializer_->init(
      measurement_manager_, measurement_buffer, &state_, &information_);
  return true;
}

}  // namespace tsif
