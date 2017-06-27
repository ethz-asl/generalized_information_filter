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

bool Estimator::defineState(std::vector<BlockType> state_types) {
  state_types_ = state_types;
  first_state_.defineState(state_types);
  return true;
}

bool Estimator::addResidual(ResidualBase* residual, std::vector<int> first_keys, std::vector<int> second_keys,
                            std::vector<int> measurement_keys) {
  CHECK(first_state_.dimension_ > 0);  // Check if state is defined.

  std::vector<Timeline*> timelines = measurement_manager_.getTimelines(measurement_keys, residual->is_mergeable_);
  residual->setMeasurementTimelines(timelines);

  problem_builder_.addResidual(residual, first_keys, second_keys, measurement_keys);

  return true;
}

void Estimator::addMeasurement(int timeline_key, int timestamp_ns, MeasurementBase* measurement) {
  if(timestamp_previous_update_ns_ > timestamp_ns) {
    std::cout << "timestamp of measurement is older than current estimator time."
        << " State buffer not yet implemented, dropping measurement: "
        << measurement->getPrintableMeasurement()
        << " at time " << std::to_string(timestamp_ns) << std::endl;
    return;
  }

  TSIF_LOG("added " << measurement->getPrintableMeasurement() << " at time " << std::to_string(timestamp_ns));
  measurement_manager_.addMeasurement(timeline_key, timestamp_ns, measurement);
  runEstimator();
}

void Estimator::runEstimator() {
  UpdateDescription update_description;
  while(measurement_manager_.updateStrategy(timestamp_previous_update_ns_, &update_description)) {
    if (first_run_) {
      // This is the first run and we have to initialize everything
      TSIF_LOG("Init filter at time " << std::to_string(update_description.timestamp_ns));
      init();
      first_run_ = false;
    }

    TSIF_LOG("update at time " << std::to_string(update_description.timestamp_ns));
    FilterProblemDescription problem_description = problem_builder_.getFilterProblemDescription(update_description);
    filter_.update(problem_description, first_state_, &first_state_);
    timestamp_previous_update_ns_ = update_description.timestamp_ns;
  }
}

void Estimator::initStateValue(const int key, const VectorXRef& value) { first_state_.setBlock(key, value); }

void Estimator::printState() const { std::cout << first_state_.printState() << std::endl; }

void Estimator::printTimeline() const { measurement_manager_.printTimeline(); }

void Estimator::printResiduals() const {
  problem_builder_.printResiduals(first_state_);
}

void Estimator::checkResiduals() {
  problem_builder_.checkResiduals(first_state_);
}

bool Estimator::init() {
  filter_.init(first_state_, problem_builder_.getTotalResidualDimension());
  return true;
}

}  // namespace tsif
