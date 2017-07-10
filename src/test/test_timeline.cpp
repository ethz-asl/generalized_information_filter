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

  timeline.addMeasurement(
      0, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));
  timeline.addMeasurement(
      10, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  int timestamp = timeline.getNextMeasurementTimestamp(0);

  EXPECT_TRUE(timestamp == 10);

  timestamp = timeline.getNextMeasurementTimestamp(10);
  EXPECT_TRUE(timestamp == -1) << timestamp;
}

TEST(TimelineTest, TimelineGetRange) {
  Timeline timeline;

  timeline.addMeasurement(
      0, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));
  timeline.addMeasurement(
      10, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  std::vector<TimedMeasurement> measurements =
      timeline.getMeasurementsInRange(5, 15);

  EXPECT_TRUE(measurements.empty());

  measurements = timeline.getMeasurementsInRange(5, 10);

  EXPECT_TRUE(measurements.size() == 2);

  EXPECT_TRUE(measurements[0].first == 0);
  EXPECT_TRUE(measurements[1].first == 10);

  timeline.addMeasurement(
      20, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  measurements = timeline.getMeasurementsInRange(5, 15);

  EXPECT_TRUE(measurements.size() == 3);

  EXPECT_TRUE(measurements[0].first == 0);
  EXPECT_TRUE(measurements[1].first == 10);
  EXPECT_TRUE(measurements[2].first == 20);
}

}  // namespace tsif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
