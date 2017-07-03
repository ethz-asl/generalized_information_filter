/*
 * measurement_manager.h
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_

#include <iostream>
#include <map>

#include "filter_test/defines.h"
#include "filter_test/helper_functions.h"
#include "filter_test/measurement.h"
#include "filter_test/timeline.h"

namespace tsif {

struct MeasurementBuffer {
  ~MeasurementBuffer() {
    if (!managed_measurements.empty()) {
      for (TimedMeasurement& measurement : managed_measurements) {
        delete measurement.second;
      }
    }
  }

  bool areMeasurementsAvailable(const std::vector<int> keys) const {
    for(const int key:keys) {
      if(timelines[key].empty()) {
        return false;
      }
    }
    return true;
  }

  std::vector<const TimedMeasurementVector*> getTimedMeasurementVectors(
      const std::vector<int> keys) const {
    std::vector<const TimedMeasurementVector*> measurement_buffers;
    for (int current_key : keys) {
      measurement_buffers.emplace_back(&(timelines[current_key]));
    }

    return measurement_buffers;
  }

  void print() const {
    if (timelines.empty())
      return;
    std::cout << "Measurement buffer start: " << timestamp_previous_update_ns
              << " end: " << timestamp_ns << std::endl;
    int i = 0;
    for (TimedMeasurementVector& timeline : timelines) {
      std::cout << "Timeline " << i << " : ";
      for (TimedMeasurement& meas : timeline) {
        std::cout << meas.first << " ";
      }
      std::cout << std::endl;
      ++i;
    }
  }

  int timestamp_ns;
  int timestamp_previous_update_ns;

  // Buffered measurements. The index is the measurement_key.
  // TODO(burrimi): Switch to map?
  std::vector<TimedMeasurementVector> timelines;

  TimedMeasurementVector managed_measurements;  // Buffer for temporary
                                                // measurements that get deleted
                                                // once out of scope.
};

class MeasurementManager {
 public:
  MeasurementManager() {}
  ~MeasurementManager() {}
  std::vector<Timeline> timelines_;
  // Takes ownership of the data.
  void addMeasurement(
      const int timeline_key, const int timestamp_ns,
      MeasurementBase* measurement);

  bool updateStrategy(
      const int& timestamp_previous_update_ns,
      MeasurementBuffer* update_description) const;

  // We assume that timeline id's start with zero. If a timeline does not exist
  // it's created.
  Timeline* getTimelinePtr(int timeline_key);
  std::vector<Timeline*> prepareTimelines(
      std::vector<int> timeline_keys, bool is_mergeable);

  void printTimeline() const;

 private:
  bool extractRelevantMeasurements(
      const Timeline& timeline, const int timestamp_start_ns,
      const int timestamp_end_ns, std::vector<TimedMeasurement>* measurements,
      std::vector<TimedMeasurement>* memory_manager) const;

  bool splitMeasurements(
      const TimedMeasurement& measurement_start,
      const TimedMeasurement& measurement_end, const int timestamp_split_ns,
      TimedMeasurement* measurement_split) const;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
