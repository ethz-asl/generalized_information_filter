/*
 * test_state.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */
#include "gtest/gtest.h"
#include "filter_test/filter.h"
#include "filter_test/measurement.h"
#include "filter_test/constant_residual.h"
#include "filter_test/constant_velocity_residual.h"
#include "filter_test/position_residual.h"

namespace tsif {

enum StateDefinition {
  kStatePosition = 0,
  kStateVelocity,
  kStateOrientation
};

enum MeasurementDefinition {
  kMeasPosition = 0,
  kMeasImu
};


TEST(FilterTest, SimpleResidualTest) {
  std::vector<BlockType> state_block_types {kVector3, kVector3, kVector2};
//  std::vector<int> state_names {kPosition, kVelocity, kOrientation};

  Filter testfilter;
  testfilter.defineState(state_block_types);

  Eigen::Vector3d test_vec(1,2,3);
  testfilter.initStateValue(kStatePosition, test_vec);

  testfilter.printState();

  ConstantResidual* test_residual = new ConstantResidual();
  std::vector<int> first_keys {kStatePosition};
  std::vector<int> second_keys {kStatePosition};
  std::vector<int> measurement_keys {kMeasImu};
  testfilter.addResidual(test_residual, first_keys, second_keys, measurement_keys);

  ConstantResidual* test_residual2 = new ConstantResidual();
  first_keys = {kStatePosition};
  second_keys = {kStateVelocity};
  testfilter.addResidual(test_residual2, first_keys, second_keys);

  PositionResidual* test_residual3 = new PositionResidual(Matrix3::Identity());
  first_keys = {};
  second_keys = {kStatePosition};
  measurement_keys = {kMeasPosition};

  testfilter.addResidual(test_residual3, first_keys, second_keys, measurement_keys);


  testfilter.printResiduals();

  testfilter.checkResiduals();


  MeasurementBase* imu_measurement1 = new ImuMeasurement(Vector3(1,1,1), Vector3(1,1,1));
  testfilter.addMeasurement(kMeasImu, 123, imu_measurement1);
  MeasurementBase* imu_measurement2 = new ImuMeasurement(Vector3(1,1,1), Vector3(1,1,1));
  testfilter.addMeasurement(kMeasImu, 130, imu_measurement2);
  MeasurementBase* position_measurement = new PositionMeasurement(Vector3(1,1,1));
  testfilter.addMeasurement(kMeasPosition, 140, position_measurement);
  MeasurementBase* imu_measurement3 = new ImuMeasurement(Vector3(1,1,1), Vector3(1,1,1));
  testfilter.addMeasurement(kMeasImu, 170, imu_measurement3);

  testfilter.printTimeline();

}

TEST(FilterTest, SimpleFilterTest) {
  std::vector<BlockType> state_block_types {kVector3, kVector3};
//  std::vector<int> state_names {kPosition, kVelocity, kOrientation};

  Filter testfilter;
  testfilter.defineState(state_block_types);

  Vector3 initial_position(1,2,3);
  testfilter.initStateValue(kStatePosition, initial_position);

  testfilter.printState();

  ConstantVelocityResidual* test_residual1 = new ConstantVelocityResidual(1,1);
  std::vector<int> first_keys {kStatePosition, kStateVelocity};
  std::vector<int> second_keys {kStatePosition, kStateVelocity};
  testfilter.addResidual(test_residual1, first_keys, second_keys);

  PositionResidual* test_residual2 = new PositionResidual(Matrix3::Identity());
  first_keys = {};
  second_keys = {kStatePosition};
  std::vector<int> measurement_keys {kMeasPosition};

  testfilter.addResidual(test_residual2, first_keys, second_keys, measurement_keys);


  testfilter.printResiduals();

  testfilter.checkResiduals();

  testfilter.printTimeline();

  MeasurementBase* position_measurement1 = new PositionMeasurement(Vector3(1,1,1));
  testfilter.addMeasurement(kMeasPosition, 140, position_measurement1);
  MeasurementBase* position_measurement2 = new PositionMeasurement(Vector3(1,1,1));
  testfilter.addMeasurement(kMeasPosition, 150, position_measurement2);

  testfilter.printTimeline();

}

}  // namespace tsif

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
