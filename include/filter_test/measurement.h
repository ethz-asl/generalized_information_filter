/*
 * measurement.h
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_MEASUREMENT_H_
#define INCLUDE_FILTER_TEST_MEASUREMENT_H_

#include <iostream>
#include <map>

#include "glog/logging.h"

#include "filter_test/defines.h"
#include "filter_test/utils/random.h"

namespace tsif {

class MeasurementBase;

using TimedMeasurement = std::pair<int64_t, MeasurementBase*>;
using TimedMeasurementMap = std::map<int64_t, MeasurementBase*>;
using TimedMeasurementVector = std::vector<TimedMeasurement>;

class MeasurementBase {
 public:
  MeasurementBase() {}

  virtual ~MeasurementBase() {}

  virtual MeasurementBase* clone() const = 0;

  virtual MeasurementBase* split(
      const MeasurementBase& next_measurement, const int64_t timestamp_ns,
      const int64_t timestamp_split_ns, const int64_t timestamp_next_ns) {
    CHECK(false) << "splitting this measurement is not implemented. Type: " +
                        getMeasurementName();
    return nullptr;
  }

  virtual std::string getMeasurementName() const = 0;

 private:
};

struct MeasurementBuffer {
  ~MeasurementBuffer() {
    if (!managed_measurements.empty()) {
      for (TimedMeasurement& measurement : managed_measurements) {
        delete measurement.second;
      }
    }
  }

  bool areMeasurementsAvailable(const std::vector<size_t>& keys) const {
    for (const size_t key : keys) {
      if (key >= timelines.size()) {
        return false;
      }
      if (timelines[key].empty()) {
        return false;
      }
    }
    return true;
  }

  std::vector<const TimedMeasurementVector*> getTimedMeasurementVectors(
      const std::vector<size_t>& keys) const {
    std::vector<const TimedMeasurementVector*> measurement_buffers;
    for (const size_t key : keys) {
      CHECK(key < timelines.size());
      measurement_buffers.emplace_back(&(timelines[key]));
    }

    return measurement_buffers;
  }

  void print() const {
    if (timelines.empty())
      return;
    std::cout << "Measurement buffer start: " << timestamp_previous_update_ns
              << " end: " << timestamp_ns << std::endl;
    size_t i = 0;
    for (const TimedMeasurementVector& timeline : timelines) {
      std::cout << "Timeline " << i << " : ";
      for (const TimedMeasurement& meas : timeline) {
        std::cout << meas.first << " ";
      }
      std::cout << std::endl;
      ++i;
    }
  }

  int64_t timestamp_ns = -1;
  int64_t timestamp_previous_update_ns = -1;

  // Buffered measurements. The index is the measurement_key.
  // TODO(burrimi): Switch to map?
  std::vector<TimedMeasurementVector> timelines;

  TimedMeasurementVector managed_measurements;  // Buffer for temporary
                                                // measurements that get deleted
                                                // once out of scope.
};

class PositionMeasurement : public MeasurementBase {
 public:
  PositionMeasurement(const Vector3& position) : position_(position) {}

  virtual ~PositionMeasurement() {}

  virtual MeasurementBase* clone() const {
    return new PositionMeasurement(
        static_cast<const PositionMeasurement&>(*this));  // call the copy ctor.
  }

  virtual std::string getMeasurementName() const {
    return "Position";
  }

  static PositionMeasurement createRandomMeasurement() {
    const Vector3 position =
        NormalRandomNumberGenerator::getInstance().template getVector<3>();
    return PositionMeasurement(position);
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
      const MeasurementBase& next_measurement, const int64_t timestamp_ns,
      const int64_t timestamp_split_ns, const int64_t timestamp_next_ns) {
    // TODO(burrimi): linear interpolation?
    return this->clone();
  }

  virtual std::string getMeasurementName() const {
    return "IMU";
  }

  static ImuMeasurement createRandomMeasurement() {
    const Vector3 acceleration =
        NormalRandomNumberGenerator::getInstance().template getVector<3>();
    const Vector3 angular_velocity =
        NormalRandomNumberGenerator::getInstance().template getVector<3>();
    return ImuMeasurement(acceleration, angular_velocity);
  }

  const Vector3 acceleration_;
  const Vector3 angular_velocity_;

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_MEASUREMENT_H_ */
