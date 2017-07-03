/*
 * measurement_manager.cpp
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */
#include "glog/logging.h"

#include "filter_test/measurement_manager.h"

namespace tsif {

// We assume that timeline id's start with zero. If a timeline does not exist it's created.
Timeline* MeasurementManager::getTimelinePtr(int timeline_key) {
  // Check if timeline for that sensor id already exists.
  while (timeline_key >= timelines_.size()) {
    timelines_.emplace_back(Timeline());
  }
  return &timelines_[timeline_key];
}

// Returns all the Timelines requested. If timeline does not exist it gets created.
std::vector<Timeline*> MeasurementManager::prepareTimelines(std::vector<int> timeline_keys, bool is_mergeable) {
  std::vector<Timeline*> timelines;
  for (int current_key : timeline_keys) {
    Timeline* current_timeline = getTimelinePtr(current_key);
    current_timeline->mergeable_ &= is_mergeable;
    timelines.emplace_back(current_timeline);
  }
  return timelines;
}

void MeasurementManager::addMeasurement(const int timeline_key, const int timestamp_ns, MeasurementBase* measurement) {
  CHECK(timeline_key < timelines_.size())
      << "Timeline key does not exist. This probably means that the measurement is not used by any residual.";
  timelines_[timeline_key].addMeasurement(timestamp_ns, measurement);
}

void MeasurementManager::printTimeline() const {
  std::vector<TimedMeasurementMap::const_iterator> timeline_iterators;
  std::vector<TimedMeasurementMap::const_iterator> timeline_iterators_end;
  int i = 0;
  int num_finished_timelines = 0;

  std::cout << padTo("", 20);
  for (const Timeline& current_timeline : timelines_) {
    timeline_iterators.push_back(current_timeline.timed_measurements_.begin());
    timeline_iterators_end.push_back(current_timeline.timed_measurements_.end());
    std::cout << padTo("M" + std::to_string(i), 5);
    if (current_timeline.isEmpty()) {
      ++num_finished_timelines;
    }
    ++i;
  }

  std::cout << std::endl;

  std::cout << padTo("mergeable?", 20);
  for (const Timeline& current_timeline : timelines_) {
    if (current_timeline.mergeable_) {
      std::cout << " yes ";
    } else {
      std::cout << " no  ";
    }
  }

  std::cout << std::endl;

  size_t num_timelines = timeline_iterators.size();
  while (num_finished_timelines < num_timelines) {
    num_finished_timelines = 0;
    int smallest_timeline = 0;
    double oldest_timestamp = std::numeric_limits<double>::max();
    for (size_t i = 0; i < num_timelines; ++i) {
      if (timeline_iterators[i] == timeline_iterators_end[i]) {
        ++num_finished_timelines;
        continue;
      }
      if (timeline_iterators[i]->first < oldest_timestamp) {
        oldest_timestamp = timeline_iterators[i]->first;
        smallest_timeline = i;
      }
    }

    // stop if all timelines are finished
    if (num_finished_timelines >= num_timelines) {
      continue;
    }

    // print nicely
    std::cout << padTo(std::to_string(oldest_timestamp), 20);
    for (size_t i = 0; i < num_timelines; ++i) {
      if (smallest_timeline == i) {
        std::cout << "  X  ";

      } else {
        std::cout << "     ";
      }
    }
    std::cout << std::endl;

    ++timeline_iterators[smallest_timeline];
  }
}

// Warning: This function allocates a new measurement which is not managed!
bool MeasurementManager::splitMeasurements(const TimedMeasurement& measurement_start, const TimedMeasurement& measurement_end, const int timestamp_split_ns, TimedMeasurement* measurement_split) const {
  // Check if we have to split
  if(measurement_start.first != timestamp_split_ns || measurement_end.first != timestamp_split_ns) {
    measurement_split->first = timestamp_split_ns;
    measurement_split->second = measurement_start.second->split(*measurement_end.second, measurement_start.first, timestamp_split_ns, measurement_end.first);
    return true;
  }
  return false;
}

// This function returns a vector of measurements in the requested range. If measurements at the boarder don't exist they are interpolated and added to the memory_manager for deallocation.
// time      x   x   x   x   x   x   x
// min/max     ^               ^
// return      x x   x   x   x x
bool MeasurementManager::extractRelevantMeasurements(const Timeline& timeline, const int timestamp_start_ns, const int timestamp_end_ns, std::vector<TimedMeasurement>* measurements, std::vector<TimedMeasurement>* memory_manager) const {
  CHECK_NOTNULL(measurements);
  CHECK_NOTNULL(memory_manager);

  std::vector<TimedMeasurement>& measurement_buffer = *measurements;
  measurement_buffer = timeline.getMeasurementsInRange(timestamp_start_ns, timestamp_end_ns);

  if(measurement_buffer.size() < 2) {
    return false;
  }

  TimedMeasurement temp_measurement;
  // Replace first measurement in buffer with split measurement
  if(splitMeasurements(measurement_buffer[0], measurement_buffer[1], timestamp_start_ns, &temp_measurement)) {
    measurement_buffer[0] = temp_measurement;
    memory_manager->emplace_back(temp_measurement);
  }

  const size_t& last_idx = measurement_buffer.size() - 1;
  // Replace last measurement in buffer with split measurement
  if(splitMeasurements(measurement_buffer[last_idx-1], measurement_buffer[last_idx], timestamp_end_ns, &temp_measurement)) {
    measurement_buffer[last_idx] = temp_measurement;
    memory_manager->emplace_back(temp_measurement);
  }
  return true;
}

bool MeasurementManager::updateStrategy(const int& timestamp_previous_update_ns, MeasurementBuffer* measurement_buffer) const {
  // Find the oldest non mergeable timestamp. In KF filtering this is the measurement.
  // TODO(burrimi): Cache this.
  double oldest_timestamp = std::numeric_limits<double>::max();
  int active_measurement_idx;
  for (size_t i = 0; i < timelines_.size(); ++i) {
    const Timeline& current_timeline = timelines_[i];
    if (current_timeline.mergeable_ || current_timeline.isEmpty()) {
      continue;
    }
    const int next_timestamp = current_timeline.getNextMeasurementTimestamp(timestamp_previous_update_ns);
    if(next_timestamp < 0) { // Check if we got a valid timestamp.
      continue;
    }
    if (next_timestamp < oldest_timestamp) {
      oldest_timestamp = next_timestamp;
      active_measurement_idx = i;
    }
  }
  if(oldest_timestamp==std::numeric_limits<double>::max()) {
    return false;
  }

  measurement_buffer->timelines.resize(timelines_.size());
  measurement_buffer->active_timeline_ids.emplace_back(active_measurement_idx);
  const TimedMeasurement& measurement = timelines_.at(active_measurement_idx).getTimedMeasurement(oldest_timestamp);
  measurement_buffer->timelines[active_measurement_idx].emplace_back(measurement);

  // TODO(burrimi): should we also check the oldest timestamp of mergeable residuals?

  // Check if all the mergeable measurements have a newer measurement to allow for interpolation.
  for (size_t i = 0; i < timelines_.size(); ++i) {
    const Timeline& current_timeline = timelines_[i];
    if (current_timeline.isEmpty()) {
      continue;
    }
    if (!current_timeline.mergeable_) {
      continue;
    }
    if (current_timeline.getNewestMeasurementTimestamp() < oldest_timestamp) {
      return false;
    }
    measurement_buffer->active_timeline_ids.emplace_back(i);
    extractRelevantMeasurements(current_timeline, timestamp_previous_update_ns, oldest_timestamp, &measurement_buffer->timelines[i], &measurement_buffer->managed_measurements);
  }
  // We probably have all the required measurements for performing one step.


  measurement_buffer->timestamp_ns = oldest_timestamp;
  measurement_buffer->timestamp_previous_update_ns = timestamp_previous_update_ns;

  return true;
}

}  // namespace tsif
