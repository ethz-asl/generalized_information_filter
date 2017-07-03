/*
 * test_state.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include "filter_test/residuals/constant_residual.h"
#include "filter_test/residuals/constant_velocity_residual.h"
#include "filter_test/residuals/position_residual.h"
#include "filter_test/estimator.h"
#include "filter_test/measurement.h"
#include "filter_test/timeline.h"
#include "gtest/gtest.h"

namespace tsif {

enum StateDefinition { kStatePosition = 0, kStateVelocity, kStateOrientation };

enum MeasurementDefinition { kMeasPosition = 0, kMeasImu };


class InitStateConstVelocity: public InitStateBase {
 public:
  InitStateConstVelocity() {}
  ~InitStateConstVelocity() {}
  virtual bool init(const MeasurementManager& measurement_manager, const UpdateDescription& update_description, State* state, MatrixX* information) {
    const Timeline& position_timeline = measurement_manager.timelines_[kMeasPosition];
    const PositionMeasurement* position_measurement = position_timeline.getMeasurement<PositionMeasurement>(update_description.timestamp_ns);

    state->template setValue<VectorBlock<3>>(kStatePosition,position_measurement->position_);
    state->template setValue<VectorBlock<3>>(kStateVelocity,Vector3::Zero());

    information->setIdentity();
    return true;
  }
 private:
};


TEST(FilterTest, TimelineTest) {
  Timeline timeline;

  timeline.addMeasurement(0, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));
  timeline.addMeasurement(10, new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1)));

  int timestamp = timeline.getNextMeasurementTimestamp(0);

  CHECK(timestamp==10);

  timestamp = timeline.getNextMeasurementTimestamp(10);
  CHECK(timestamp==-1) << timestamp;
}

TEST(FilterTest, SimpleResidualTest) {
  std::vector<BlockType> state_block_types{kVector3, kVector3, kVector2};
  //  std::vector<int> state_names {kPosition, kVelocity, kOrientation};

  Estimator testfilter(new DummyInitState());
  testfilter.defineState(state_block_types);

  Eigen::Vector3d test_vec(1, 2, 3);
  testfilter.initStateValue(kStatePosition, test_vec);

  testfilter.printState();

  ConstantResidual* test_residual = new ConstantResidual();
  std::vector<int> first_keys{kStatePosition};
  std::vector<int> second_keys{kStatePosition};
  std::vector<int> measurement_keys{kMeasImu};
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

  MeasurementBase* imu_measurement1 = new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1));
  testfilter.addMeasurement(kMeasImu, 123, imu_measurement1);
  MeasurementBase* imu_measurement2 = new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1));
  testfilter.addMeasurement(kMeasImu, 130, imu_measurement2);
  MeasurementBase* position_measurement = new PositionMeasurement(Vector3(1, 1, 1));
  testfilter.addMeasurement(kMeasPosition, 140, position_measurement);
  MeasurementBase* imu_measurement3 = new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1));
  testfilter.addMeasurement(kMeasImu, 170, imu_measurement3);
  MeasurementBase* position_measurement2 = new PositionMeasurement(Vector3(1, 1, 1));
  testfilter.addMeasurement(kMeasPosition, 180, position_measurement2);
  MeasurementBase* imu_measurement4 = new ImuMeasurement(Vector3(1, 1, 1), Vector3(1, 1, 1));
  testfilter.addMeasurement(kMeasImu, 190, imu_measurement4);
  testfilter.printTimeline();
}

TEST(FilterTest, SimpleFilterTest) {
  std::vector<BlockType> state_block_types{kVector3, kVector3};
  //  std::vector<int> state_names {kPosition, kVelocity, kOrientation};

  Estimator testfilter(new InitStateConstVelocity());
  testfilter.defineState(state_block_types);

  ConstantVelocityResidual* test_residual1 = new ConstantVelocityResidual(1, 1);
  std::vector<int> first_keys{kStatePosition, kStateVelocity};
  std::vector<int> second_keys{kStatePosition, kStateVelocity};
  testfilter.addPredictionResidual(test_residual1, first_keys, second_keys);

  PositionResidual* test_residual2 = new PositionResidual(Matrix3::Identity());
  first_keys = {};
  second_keys = {kStatePosition};
  std::vector<int> measurement_keys{kMeasPosition};

  testfilter.addResidual(test_residual2, first_keys, second_keys, measurement_keys);

  testfilter.printResiduals();

  testfilter.checkResiduals();

  testfilter.printTimeline();

  MeasurementBase* position_measurement1 = new PositionMeasurement(Vector3(1.005, 2.005, 3.005));
  testfilter.addMeasurement(kMeasPosition, 5000000, position_measurement1);

  MeasurementBase* position_measurement2 = new PositionMeasurement(Vector3(1.010, 2.010, 3.010));
  testfilter.addMeasurement(kMeasPosition, 10000000, position_measurement2);

  testfilter.printTimeline();
}

}  // namespace tsif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
