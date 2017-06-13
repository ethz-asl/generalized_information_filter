/*
 * measurement_manager.cpp
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */

#include "filter_test/measurement_manager.h"

// We assume that timeline id's start with zero. If a timeline does not exist it's created.
Timeline* MeasurementManager::getTimelinePtr(int timeline_key) {
  // Check if timeline for that sensor id already exists.
  while(timeline_key >= timelines_.size()) {
    timelines_.emplace_back(Timeline());
  }
  return &timelines_[timeline_key];
}

// Returns all the Timelines requested. If timeline does not exist it gets created.
std::vector<Timeline*> MeasurementManager::getTimelines(std::vector<int> timeline_keys, bool is_mergeable) {
  std::vector<Timeline*> timelines;
  for(int current_key:timeline_keys) {
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

void MeasurementManager::printTimeline() {
  std::vector<TimedMeasurement::const_iterator> timeline_iterators;
  std::vector<TimedMeasurement::const_iterator> timeline_iterators_end;
  int i=0;
  int num_finished_timelines = 0;

  std::cout << padTo("", 20);
  for(const Timeline& current_timeline:timelines_) {
    timeline_iterators.push_back(current_timeline.timed_measurements_.begin());
    timeline_iterators_end.push_back(current_timeline.timed_measurements_.end());
    std::cout << padTo("M" + std::to_string(i), 5);
    if(current_timeline.isEmpty()) {
      ++num_finished_timelines;
    }
    ++i;
  }

  std::cout << std::endl;


  std::cout << padTo("mergeable?", 20);
  for(const Timeline& current_timeline:timelines_) {
    if(current_timeline.mergeable_) {
      std::cout << "  X  ";
    }else{
      std::cout << "     ";
    }
  }

  std::cout << std::endl;

  size_t num_timelines = timeline_iterators.size();
  while(num_finished_timelines < num_timelines)
  {
    num_finished_timelines = 0;
    int smallest_timeline = 0;
    double oldest_timestamp = std::numeric_limits<double>::max();
    for(size_t i=0;i<num_timelines;++i) {
      if(timeline_iterators[i] == timeline_iterators_end[i]) {
        ++num_finished_timelines;
        continue;
      }
      if(timeline_iterators[i]->first < oldest_timestamp) {
        oldest_timestamp = timeline_iterators[i]->first;
        smallest_timeline = i;
      }
    }

    // stop if all timelines are finished
    if(num_finished_timelines >= num_timelines) {
      continue;
    }

    // print nicely
    std::cout << padTo(std::to_string(oldest_timestamp), 20);
    for(size_t i=0;i<num_timelines;++i) {
      if(smallest_timeline==i) {
        std::cout << "  X  ";

      }else{
        std::cout << "     ";
      }
    }
    std::cout << std::endl;

    ++timeline_iterators[smallest_timeline];
  }
}

bool MeasurementManager::shouldIRunTheFilter(const int& timestamp_previous_update_ns, int* timestamp_update_ns) {
  // Find the oldest non mergeable timestamp.
  // TODO(burrimi): Cache this.
  double oldest_timestamp = std::numeric_limits<double>::max();
  for(const Timeline& current_timeline:timelines_) {
    if(current_timeline.mergeable_ || current_timeline.isEmpty()) {
      continue;
    }
    const double& next_timestamp = current_timeline.getNextMeasurementTimestamp(timestamp_previous_update_ns);
    if(next_timestamp < oldest_timestamp) {
      oldest_timestamp = next_timestamp;
    }
  }
  // TODO(burrimi): should we also check the oldest timestamp of mergeable residuals?

  // Check if all the mergeable measurements have a newer measurement to allow for interpolation.
  for(const Timeline& current_timeline:timelines_) {
    if(current_timeline.isEmpty()) {
      continue;
    }
    if(!current_timeline.mergeable_) {
      continue;
    }
    if(current_timeline.getNewestMeasurementTimestamp() < oldest_timestamp) {
      return false;
    }
  }
  // We probably have all the required measurements for performing one step.
  *timestamp_update_ns = oldest_timestamp;
  return true;
}
