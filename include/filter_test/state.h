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
    for (BlockBase* current_block : state_blocks_) {
      delete current_block;
    }
  }

  // Copy constructor clones the other state and takes care of allocating the
  // new blocks
  State(const State& other) : dimension_(0), minimal_dimension_(0) {
    cloneState(other);
  }

  State& operator=(const State& other) {
    // check for self-assignment
    if (&other == this)
      return *this;

    // reuse storage when possible
    // TODO(burrimi): Do better check! For now we assume that the same dimension
    // is probably the same state :/
    if (dimension_ == other.dimension_) {
      BlockVector::iterator block_iterator = state_blocks_.begin();
      BlockVector::const_iterator other_block_iterator =
          other.state_blocks_.begin();
      while (other_block_iterator != other.state_blocks_.end()) {
        (*other_block_iterator)->copyBlockTo(*block_iterator);
        ++block_iterator;
        ++other_block_iterator;
      }
      return *this;
    }

    // At this point the state is probably empty and we have to allocate the
    // blocks
    cloneState(other);
    return *this;
  }

  inline void cloneState(const State& other) {
    for (BlockBase* current_block : other.state_blocks_) {
      BlockBase* new_block = current_block->clone();
      addBlock(new_block);
    }
  }

  void defineState(std::vector<BlockType> block_types) {
    for (BlockType current_type : block_types) {
      BlockBase* current_block = block_helper::createBlockByType(current_type);
      addBlock(current_block);
    }
  }

  void setBlock(const int key, const VectorXRef& value) {
    getBlock(key)->setValue(value);
  }

  template <typename BlockType>
  void setValue(const int key, const typename BlockType::StorageType& value) {
    BlockType* block = dynamic_cast<BlockType*>(getBlock(key));
    CHECK_NOTNULL(block);  // Check if cast successful
    block->setValue(value);
  }

  template <typename BlockType>
  typename BlockType::StorageType& getValue(const int key) {
    BlockType* block = getBlock(key);
    return block->template getValue<BlockType>();
  }

  inline BlockBase* getBlock(const int& key) const {
    CHECK(state_blocks_.size() > key);  // Check if key is valid.
    return state_blocks_[key];
  }

  inline std::vector<BlockBase*> getBlocks(const std::vector<int>& keys) const {
    std::vector<BlockBase*> blocks;
    for (const int& current_key : keys) {
      blocks.emplace_back(getBlock(current_key));
    }
    return blocks;
  }

  template <typename BlockType>
  BlockType* getBlock(const int& key) const {
    BlockType* block = dynamic_cast<BlockType*>(getBlock(key));
    CHECK_NOTNULL(block);  // Check if key is valid.
    return block;
  }

  size_t numberOfBlocks() const {
    return state_blocks_.size();
  }

  void boxPlus(const VectorXRef& dx, State* result_state) const {
    CHECK_NOTNULL(result_state);
    CHECK(
        minimal_dimension_ == dx.size());  // Check if dimension of dx is valid.

    // Convenience function: if we have an empty state we clone the current
    // state to create it.
    if (result_state->dimension_ == 0) {
      result_state->cloneState(*this);
    }

    CHECK(dimension_ == result_state->dimension_)
        << "Dimension of first state" << dimension_ << " result state "
        << result_state
               ->dimension_;  // Check if dimension of result_state is valid.
    int accumulated_dimension = 0;

    BlockVector::iterator block_iterator = result_state->state_blocks_.begin();
    for (BlockBase* current_block : state_blocks_) {
      current_block->boxPlus(
          dx.segment(accumulated_dimension, current_block->minimal_dimension_),
          (*block_iterator));
      accumulated_dimension += current_block->minimal_dimension_;
      ++block_iterator;
    }
  }

  // calculates this - other = dx
  void boxMinus(const State& other, VectorX* dx) const {
    //    CHECK_NOTNULL(dx);
    CHECK(
        minimal_dimension_ ==
        dx->size());  // Check if dimension of dx is valid.

    CHECK(dimension_ == other.dimension_)
        << "Dimension of first state " << dimension_ << " other state "
        << other.dimension_;  // Check if dimension of result_state is valid.

    int index = 0;
    BlockVector::const_iterator block_iterator = other.state_blocks_.begin();
    for (BlockBase* current_block : state_blocks_) {
      dx->segment(index, current_block->minimal_dimension_) =
          current_block->boxMinus((*block_iterator));
      index += current_block->minimal_dimension_;
      ++block_iterator;
    }
  }

  inline int getAccumulatedMinimalDimension(const int& key) const {
    return accumulated_minimal_dimensions_[key];
  }

  VectorX getAsVector() const {
    VectorX state_vector(dimension_);
    int index = 0;
    for (BlockBase* current_block : state_blocks_) {
      state_vector.segment(index, current_block->dimension_) =
          current_block->getValueAsVector();
      index += current_block->dimension_;
    }
    return state_vector;
  }

  std::string print() const {
    std::ostringstream oss;
    for (BlockBase* current_block : state_blocks_) {
      oss << current_block->getTypeName() << "["
          << current_block->getValueAsVector().transpose() << "], ";
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
