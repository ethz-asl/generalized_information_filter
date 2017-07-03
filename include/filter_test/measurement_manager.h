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

struct UpdateDescription {
  ~UpdateDescription() {
     if(!managed_measurements.empty()) {
       for(TimedMeasurement& measurement:managed_measurements) {
         delete measurement.second;
       }
     }
  }

  std::vector<const TimedMeasurementVector*> getTimedMeasurementVectors(const std::vector<int> keys) const{
    std::vector<const TimedMeasurementVector*> measurement_buffers;
    for(int current_key:keys) {
      measurement_buffers.emplace_back(&(timelines[current_key]));
    }

    return measurement_buffers;
  }

  void print() {
    if(timelines.empty()) return;
    std::cout << "Update description start: " << timestamp_previous_update_ns << " end: " << timestamp_ns << std::endl;
    for(TimedMeasurementVector& timeline:timelines) {
      std::cout << "S: ";
       for(TimedMeasurement& meas:timeline) {
         std::cout << meas.first << " ";
       }
       std::cout << std::endl;
    }
  }
  int timestamp_ns;
  int timestamp_previous_update_ns;
  std::vector<int> active_timeline_ids;
  TimedMeasurementVector managed_measurements;

  std::vector<TimedMeasurementVector> timelines;
};


class MeasurementManager {
 public:
  MeasurementManager() {}
  ~MeasurementManager() {}
  std::vector<Timeline> timelines_;
  // Takes ownership of the data.
  void addMeasurement(const int timeline_key, const int timestamp_ns, MeasurementBase* measurement);



  bool updateStrategy(const int& timestamp_previous_update_ns, UpdateDescription* update_description) const;

  // We assume that timeline id's start with zero. If a timeline does not exist it's created.
  Timeline* getTimelinePtr(int timeline_key);
  std::vector<Timeline*> prepareTimelines(std::vector<int> timeline_keys, bool is_mergeable);

  void printTimeline() const;

 private:
  bool extractRelevantMeasurements(const Timeline& timeline, const int timestamp_start_ns, const int timestamp_end_ns, std::vector<TimedMeasurement>* measurements, std::vector<TimedMeasurement>* memory_manager) const;

  bool splitMeasurements(const TimedMeasurement& measurement_start, const TimedMeasurement& measurement_end, const int timestamp_split_ns, TimedMeasurement* measurement_split) const;

};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
