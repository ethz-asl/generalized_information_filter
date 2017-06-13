/*
 * measurement_manager.h
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_

#include <map>
#include <iostream>

#include "filter_test/defines.h"
#include "filter_test/measurement.h"
#include "filter_test/helper_functions.h"

using TimedMeasurement = std::map<int, MeasurementBase*>;
using TimedMeasurement = std::map<int, MeasurementBase*>;

class Timeline {

 public:
  // If a timeline is not mergeable then we have to apply the measurement before continuing.
  // This is usually the case for traditional measurements in a Kalman filter (they are not mergeable).
  bool mergeable_ = true;
  TimedMeasurement timed_measurements_;
  void addMeasurement(const int timestamp_ns, MeasurementBase* measurement) {
    while(kMaxMeasurmenetBufferSize <= timed_measurements_.size()) { // remove oldest if buffer is full
      deleteMeasurement(timed_measurements_.begin());
    }
    timed_measurements_.emplace_hint(timed_measurements_.end(), timestamp_ns, measurement);
  }
  void deleteOlderThen(const int timestamp_ns); //TODO(burrimi):implement.
  ~Timeline() {
    // Clean up measurement buffer.
    for(auto& timed_measurement:timed_measurements_) {
      delete timed_measurement.second;
    }
  }
  inline int getOldestMeasurementTimestamp() const{
    return timed_measurements_.begin()->first;
  }
  inline int getNextMeasurementTimestamp(const int timestamp_ns) const{
    const auto it = timed_measurements_.upper_bound(timestamp_ns);
    if(it!=timed_measurements_.end()) {
      return it->first;
    }
    return -1;
  }

  inline const MeasurementBase* getMeasurement(const int timestamp_ns) const{
    const auto& timed_measurement = timed_measurements_.find(timestamp_ns);
    if(timed_measurement != timed_measurements_.end()) {
      return timed_measurement->second;
    }
    return NULL;
  }

  inline int getNewestMeasurementTimestamp() const{
    return timed_measurements_.rbegin()->first;
  }
  inline int getNumMeasurements() const {
    return timed_measurements_.size();
  }
  inline bool isEmpty() const {
    return timed_measurements_.empty();
  }
  inline void deleteMeasurement(const TimedMeasurement::iterator& iterator) {
    delete iterator->second; // Free memory.
    timed_measurements_.erase(iterator); // Remove from buffer.
  }
};


class MeasurementManager {
public:
  MeasurementManager() {}
  ~MeasurementManager() {}
  std::vector<Timeline> timelines_;
  // Takes ownership of the data.
  void addMeasurement(const int timeline_key, const int timestamp_ns, MeasurementBase* measurement);

  bool shouldIRunTheFilter(const int& timestamp_previous_update_ns, int* timestamp_update_ns);

  Timeline* getTimelinePtr(int timeline_key);
  std::vector<Timeline*> getTimelines(std::vector<int> timeline_keys, bool is_mergeable);

  void printTimeline();


private:
};

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
