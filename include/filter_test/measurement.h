/*
 * measurement.h
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_H_

#include "glog/logging.h"

#include "filter_test/defines.h"

namespace tsif {

class MeasurementBase {
 public:
  MeasurementBase() {}

  virtual ~MeasurementBase() {}

  virtual MeasurementBase* clone() const = 0;

  virtual MeasurementBase* split(
      const MeasurementBase& next_measurement, const int timestamp_ns,
      const int timestamp_split_ns, const int timestamp_next_ns) {
    CHECK(false) << "splitting this measurement is not implemented. Type: " +
                        getPrintableMeasurement();
    return nullptr;
  }

  virtual std::string getPrintableMeasurement() const = 0;

 private:
};

class PositionMeasurement : public MeasurementBase {
 public:
  PositionMeasurement(const Vector3& position) : position_(position) {}

  virtual ~PositionMeasurement() {}

  virtual MeasurementBase* clone() const {
    return new PositionMeasurement(
        static_cast<const PositionMeasurement&>(*this));  // call the copy ctor.
  }

  virtual std::string getPrintableMeasurement() const {
    return "Position";
  }
  const Vector3 position_;

 private:
};

class ImuMeasurement : public MeasurementBase {
 public:
  ImuMeasurement(const Vector3& acceleration, const Vector3& angular_velocity)
      : acceleration_(acceleration), angular_velocity_(angular_velocity) {}

  virtual ~ImuMeasurement() {}

  virtual MeasurementBase* clone() const {
    return new ImuMeasurement(
        static_cast<const ImuMeasurement&>(*this));  // call the copy ctor.
  }

  virtual MeasurementBase* split(
      const MeasurementBase& next_measurement, const int timestamp_ns,
      const int timestamp_split_ns, const int timestamp_next_ns) {
    // TODO(burrimi): linear interpolation?
    return this->clone();
  }

  virtual std::string getPrintableMeasurement() const {
    return "IMU";
  }
  const Vector3 acceleration_;
  const Vector3 angular_velocity_;

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_H_ */
