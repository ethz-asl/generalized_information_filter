/*
 * measurement.h
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_H_

#include "filter_test/block.h"

namespace tsif {

class MeasurementBase {
 public:
  MeasurementBase() {}

  virtual ~MeasurementBase() {}

  //  virtual MeasurementBase* interpolateMeasurements() = 0;

  virtual std::string getPrintableMeasurement() const = 0;
 private:
};


class PositionMeasurement: public MeasurementBase {
 public:
  PositionMeasurement(const Vector3& position):position_(position) {}

  virtual ~PositionMeasurement() {}

  //  virtual MeasurementBase* interpolateMeasurements() = 0;

  virtual std::string getPrintableMeasurement() const {
    return "Position";
  }
  const Vector3 position_;
 private:
};

class ImuMeasurement: public MeasurementBase {
 public:
  ImuMeasurement(const Vector3& acceleration, const Vector3& angular_velocity):acceleration_(acceleration), angular_velocity_(angular_velocity) {}

  virtual ~ImuMeasurement() {}

  //  virtual MeasurementBase* interpolateMeasurements() = 0;

  virtual std::string getPrintableMeasurement() const {
    return "IMU";
  }
  const Vector3 acceleration_;
  const Vector3 angular_velocity_;
 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_H_ */
