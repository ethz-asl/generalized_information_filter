/*
 * test_state.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */
#include "gtest/gtest.h"
#include "filter_test/filter.h"
#include "filter_test/constant_residual.h"

enum StateDefinition {
  kStatePosition = 0,
  kStateVelocity,
  kStateOrientation
};

enum MeasurementDefinition {
  kMeasPosition = 0,
  kMeasImu
};

TEST(StateTest, AssembleState) {
  EXPECT_TRUE(true);
}

TEST(StateTest, SimpleResidualTest) {
  std::vector<BlockType> state_block_types {kVector3, kVector3, kVector2};
//  std::vector<int> state_names {kPosition, kVelocity, kOrientation};

  Filter testfilter;
  testfilter.defineState(state_block_types);

  Eigen::Vector3d test_vec(1,2,3);
  testfilter.initStateValue(kStatePosition, test_vec);

//  testfilter.printState();

  ConstantResidual* test_residual = new ConstantResidual();
  std::vector<int> first_keys {kStatePosition};
  std::vector<int> second_keys {kStatePosition};
  testfilter.addResidual(test_residual, first_keys, second_keys);

  ConstantResidual* test_residual2 = new ConstantResidual();
  first_keys = {kStatePosition};
  second_keys = {kStateVelocity};
  testfilter.addResidual(test_residual2, first_keys, second_keys);


  testfilter.checkResiduals();

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
