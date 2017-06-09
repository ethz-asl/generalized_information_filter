/*
 * measurement_manager.h
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_

#include <map>

#include "filter_test/measurement.h"

class Timeline {
public:
  bool mergeable_;
  std::map<double, MeasurementBase*> timed_measurements_;
private:
};


class MeasurementManager {
public:
  MeasurementManager() {}
  ~MeasurementManager() {
    // Clean up measurement buffers.
    for(Timeline& timeline:timelines_) {
      for(auto& timed_measurement:timeline.timed_measurements_) {
        delete timed_measurement.second;
      }
    }
  }
  std::vector<Timeline> timelines_;
  // Takes ownership of the data.
  void addMeasurement(double tiemstamp, MeasurementBase* measurement_);

private:
};

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_MANAGER_H_ */
