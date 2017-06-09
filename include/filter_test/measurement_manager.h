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

  using TimedMeasurement = std::map<double, MeasurementBase*>;


class Timeline {

 public:
  // If a timeline is not mergeable then we have to apply the measurement before continuing.
  // This is usually the case for traditional measurements in a Kalman filter (they are not mergeable).
  bool mergeable_ = true;
  TimedMeasurement timed_measurements_;
  void addMeasurement(double timestamp, MeasurementBase* measurement) {
    while(kMaxMeasurmenetBufferSize <= timed_measurements_.size()) { // remove oldest if buffer is full
      timed_measurements_.erase(timed_measurements_.begin());
    }
    timed_measurements_.emplace_hint(timed_measurements_.end(), timestamp, measurement);
  }
  void deleteOlderThen(double timestamp); //TODO(burrimi):implement.
  ~Timeline() {
    for(auto& timed_measurement:timed_measurements_) {
      delete timed_measurement.second;
    }
  }
  inline double getOldestMeasurementTimestamp() const{
    return timed_measurements_.begin()->first;
  }
  inline double getNewestMeasurementTimestamp() const{
    return timed_measurements_.rbegin()->first;
  }
  inline int getNumMeasurements() const {
    return timed_measurements_.size();
  }
  inline bool isEmpty() const {
    return timed_measurements_.empty();
  }
};


class MeasurementManager {
public:
  MeasurementManager() {}
  ~MeasurementManager() {}
  std::vector<Timeline> timelines_;
  // Takes ownership of the data.
  void addMeasurement(int timeline_key, double timestamp, MeasurementBase* measurement);

  bool shouldIRunTheFilter();

  Timeline* getTimelinePtr(int timeline_key);
  std::vector<Timeline*> getTimelines(std::vector<int> timeline_keys, bool is_mergeable);

  void printTimeline();


private:
};

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
