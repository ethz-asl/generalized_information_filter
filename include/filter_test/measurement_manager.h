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

class MeasurementManager {
 public:
  MeasurementManager() {}
  ~MeasurementManager() {}

  // Takes ownership of the data.
  void addMeasurement(
      const int timeline_key, const int64_t timestamp_ns,
      MeasurementBase* measurement);

  bool updateStrategy(
      const int64_t timestamp_previous_update_ns,
      MeasurementBuffer* update_description) const;

  // We assume that timeline id's start with zero. If a timeline does not exist
  // it's created.
  Timeline* getTimelinePtr(int timeline_key);
  std::vector<Timeline*> prepareTimelines(
      const std::vector<int>& timeline_keys, const bool is_mergeable);

  void printTimeline() const;

  inline const Timeline& getTimeline(const int key) const {
    CHECK(key < timelines_.size());
    return timelines_[key];
  }

 private:
  bool extractRelevantMeasurements(
      const Timeline& timeline, const int64_t timestamp_start_ns,
      const int64_t timestamp_end_ns,
      std::vector<TimedMeasurement>* measurements,
      std::vector<TimedMeasurement>* memory_manager) const;

  bool splitMeasurements(
      const TimedMeasurement& measurement_start,
      const TimedMeasurement& measurement_end, const int64_t timestamp_split_ns,
      TimedMeasurement* measurement_split) const;

  std::vector<Timeline> timelines_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
