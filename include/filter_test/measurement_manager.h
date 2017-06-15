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
  std::vector<Timeline> timelines_;
  // Takes ownership of the data.
  void addMeasurement(const int timeline_key, const int timestamp_ns, MeasurementBase* measurement);

  bool shouldIRunTheFilter(const int& timestamp_previous_update_ns, int* timestamp_update_ns) const;

  // We assume that timeline id's start with zero. If a timeline does not exist it's created.
  Timeline* getTimelinePtr(int timeline_key);
  std::vector<Timeline*> getTimelines(std::vector<int> timeline_keys, bool is_mergeable);

  void printTimeline() const;

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
