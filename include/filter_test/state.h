/*
 * state.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_STATE_H_
#define INCLUDE_FILTER_TEST_STATE_H_

#include "filter_test/block.h"

class State {
 public:
  std::vector<BlockBase*> state_blocks_;
  int dimension_;
  int minimal_dimension_;

  State() : dimension_(0), minimal_dimension_(0) {}

  ~State() {
    for(BlockBase* current_block: state_blocks_ ) {
      delete current_block;
    }
  }

  void defineState(std::vector<BlockType> block_types) {
    for(BlockType current_type: block_types ) {
      BlockBase* current_block = block_helper::createBlockByType(current_type);
      dimension_ += current_block->dimension_;
      minimal_dimension_ += current_block->minimal_dimension_;
      state_blocks_.push_back(current_block);
    }
  }

  void setState(const int key, const VectorXRef& value) {
    CHECK(state_blocks_.size() > key); // Check if key is valid.
    state_blocks_.at(key)->setValue(value);
  }

  void boxPlus(const VectorXRef& dx, State* result_state) {
    CHECK(minimal_dimension_ == dx.size()); // Check if dimension of dx is valid.
    CHECK(dimension_ == result_state->dimension_); // Check if dimension of result_state is valid.
    for(BlockBase* current_block: state_blocks_ ) {
      CHECK(false); //TODO(burrimi): implement.
    }
  }

  void printState() {
    for(BlockBase* current_block: state_blocks_ ) {
      std::cout << current_block->getTypeName() << "[" << current_block->getValue().transpose() << "], ";
    }
    std::cout << std::endl;
  }
 private:
};



#endif /* INCLUDE_FILTER_TEST_STATE_H_ */
