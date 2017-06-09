/*
 * measurement.h
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_H_

#include "filter_test/block.h"

class MeasurementBase {
 public:
  MeasurementBase() {}

  virtual ~MeasurementBase() {}

//  virtual MeasurementBase* interpolateMeasurements() = 0;

  virtual std::string getPrintableMeasurement() = 0;
 private:
};




#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_H_ */
