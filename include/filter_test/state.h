/*
 * state.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_STATE_H_
#define INCLUDE_FILTER_TEST_STATE_H_

#include "filter_test/block.h"

namespace tsif {

class State {
  using BlockVector = std::vector<BlockBase*>;
 public:
  BlockVector state_blocks_;
  std::vector<int> accumulated_minimal_dimensions_;
  int dimension_;
  int minimal_dimension_;

  State() : dimension_(0), minimal_dimension_(0) {}

  ~State() {
    for(BlockBase* current_block: state_blocks_ ) {
      delete current_block;
    }
  }

  // Copy constructor clones the other state and takes care of allocating the new blocks
  State(const State& other) : dimension_(0), minimal_dimension_(0) {
    for(BlockBase* current_block: other.state_blocks_ ) {
      BlockBase* new_block = current_block->clone();
      addBlock(new_block);
    }
  }

  State& operator= (const State& other)
  {
    // check for self-assignment
    if(&other == this)
      return *this;

    // reuse storage when possible
    // TODO(burrimi): Do better check! For now we assume that the same dimension is probably the same state :/
    if(dimension_ == other.dimension_) {
      BlockVector::iterator block_iterator = state_blocks_.begin();
      BlockVector::const_iterator  other_block_iterator = other.state_blocks_.begin();
      while(other_block_iterator!=other.state_blocks_.end()) {
        (*other_block_iterator)->copyBlockTo(*block_iterator);
        ++block_iterator;
        ++other_block_iterator;
      }
      return *this;
    }

    // At this point the state is probably empty and we have to allocate the blocks
    for(BlockBase* current_block: other.state_blocks_ ) {
      BlockBase* new_block = current_block->clone();
      addBlock(new_block);
    }
    return *this;
  }

  void defineState(std::vector<BlockType> block_types) {
    for(BlockType current_type: block_types ) {
      BlockBase* current_block = block_helper::createBlockByType(current_type);
      addBlock(current_block);
    }
  }

  void setState(const int key, const VectorXRef& value) {
    CHECK(state_blocks_.size() > key); // Check if key is valid.
    state_blocks_.at(key)->setValue(value);
  }

  void boxPlus(const VectorXRef& dx, State* result_state) const {
    CHECK(minimal_dimension_ == dx.size()); // Check if dimension of dx is valid.
    CHECK(dimension_ == result_state->dimension_) << "Dimension of first state" <<
        dimension_ << " result state " << result_state->dimension_; // Check if dimension of result_state is valid.
    int accumulated_dimension = 0;

    BlockVector::iterator block_iterator = result_state->state_blocks_.begin();
    for(BlockBase* current_block: state_blocks_ ) {
      current_block->boxPlus(dx.segment(accumulated_dimension, current_block->minimal_dimension_), (*block_iterator));
      accumulated_dimension += current_block->minimal_dimension_;
      ++block_iterator;
    }
  }

  inline int getAccumulatedMinimalDimension(const int& key) const {
    return accumulated_minimal_dimensions_[key];
  }

  std::string printState() const {
    std::ostringstream oss;
    for(BlockBase* current_block: state_blocks_ ) {
      oss << current_block->getTypeName() << "[" << current_block->getValue().transpose() << "], ";
    }
    return oss.str();
  }

  // note
 private:
  inline void addBlock(BlockBase* block_to_add) {
    accumulated_minimal_dimensions_.push_back(dimension_);
    dimension_ += block_to_add->dimension_;
    minimal_dimension_ += block_to_add->minimal_dimension_;
    state_blocks_.push_back(block_to_add);
  }

};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_STATE_H_ */
