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

using namespace tsif;

TEST(StateTest, DefineState) {
  std::vector<BlockType> state_block_types {kVector3, kVector1, kVector2};
  State first_state;
  first_state.defineState(state_block_types);

  EXPECT_TRUE(first_state.dimension_ ==6);
  EXPECT_TRUE(first_state.minimal_dimension_ ==6);
}

TEST(StateTest, SetState) {
  std::vector<BlockType> state_block_types {kVector3, kVector1, kVector2};

  State first_state;
  first_state.defineState(state_block_types);

  Vector3 test(1,2,1);
  first_state.setState(kStatePosition, test);

  std::cout << first_state.printState() << std::endl;

  VectorX state_vector_expected(6);
  state_vector_expected << 1,2,1,0,0,0;

  EXPECT_TRUE(first_state.getAsVector()==state_vector_expected);


  VectorX vector_2(2);
  vector_2 << 3,5;
  first_state.setState(kStateOrientation, vector_2);

  state_vector_expected << 1,2,1,0,3,5;
  EXPECT_TRUE(first_state.getAsVector()==state_vector_expected);
}

TEST(StateTest, CopyAndAssignOperator) {
  std::vector<BlockType> state_block_types {kVector3, kVector1, kVector2};

  State first_state;
  first_state.defineState(state_block_types);

  Vector3 test(1,2,1);
  first_state.setState(kStatePosition, test);

  VectorX state_vector_expected(6);
  state_vector_expected << 1,2,1,0,0,0;

  // Check copy constructor.
  State second_state = first_state;
  EXPECT_TRUE(second_state.getAsVector()==state_vector_expected);

  // Check assignment operator.
  State third_state;
  third_state = first_state;
  EXPECT_TRUE(third_state.getAsVector()==state_vector_expected);
}

}  // namespace tsif


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
