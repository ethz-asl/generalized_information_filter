/*
 * residual.cpp
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#include "filter_test/residual.h"


bool ResidualBase::inputTypesValid(const std::vector<BlockBase*>& state1,
                                   const std::vector<BlockBase*>& state2) {
  CHECK(state1.size() == state1_block_types_.size()) << "Number of blocks for first state not correct!!!";
  std::vector<BlockType>::iterator current_state_type_it = state1_block_types_.begin();
  for(BlockBase* current_block:state1) {
    CHECK(*current_state_type_it == current_block->type_)  << "Wrong block type detected!!!";
    ++current_state_type_it;
  }

  CHECK(state2.size() == state2_block_types_.size()) << "Number of blocks for second state not correct!!!";
  current_state_type_it = state2_block_types_.begin();
  for(BlockBase* current_block:state2) {
    CHECK(*current_state_type_it == current_block->type_)  << "Wrong block type detected!!!";
    ++current_state_type_it;
  }
}

void ResidualBase::setMeasurementTimelines(std::vector<Timeline*> timelines) {
  measurement_timelines_ = timelines;
}
