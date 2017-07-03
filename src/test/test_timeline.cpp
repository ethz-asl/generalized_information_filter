/*
 * test_timeline.cpp
 *
 *  Created on: 28.06.2017
 *      Author: burrimi
 */




/*
 * test_state.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include "filter_test/measurement.h"
#include "filter_test/timeline.h"
#include "gtest/gtest.h"

namespace tsif {


TEST(TimelineTest, TimelineGetNextMeasurement) {
  Timeline timeline;

  timeline.addMeasurement(0, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));
  timeline.addMeasurement(10, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  int timestamp = timeline.getNextMeasurementTimestamp(0);

  CHECK(timestamp==10);

  timestamp = timeline.getNextMeasurementTimestamp(10);
  CHECK(timestamp==-1) << timestamp;
}

TEST(TimelineTest, TimelineGetRange) {
  Timeline timeline;

  timeline.addMeasurement(0, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));
  timeline.addMeasurement(10, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  std::vector<TimedMeasurement> measurements = timeline.getMeasurementsInRange(5,15);

  CHECK(measurements.empty());

  measurements = timeline.getMeasurementsInRange(5,10);

  CHECK(measurements.size()==2);

  CHECK(measurements[0].first==0);
  CHECK(measurements[1].first==10);

  timeline.addMeasurement(20, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  measurements = timeline.getMeasurementsInRange(5,15);

  CHECK(measurements.size()==3);

  CHECK(measurements[0].first==0);
  CHECK(measurements[1].first==10);
  CHECK(measurements[2].first==20);

}


}  // namespace tsif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
