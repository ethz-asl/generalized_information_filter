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
  using BlockVector = std::vector<BlockBase::Ptr>;

 public:
  int dimension_;
  int minimal_dimension_;

  State() : dimension_(0), minimal_dimension_(0) {}

  ~State() {}

  // Copy constructor clones the other state and takes care of allocating the
  // new blocks
  State(const State& other) : dimension_(0), minimal_dimension_(0) {
    copyFrom(other);
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
        (*other_block_iterator)->copyBlockTo((*block_iterator).get());
        ++block_iterator;
        ++other_block_iterator;
      }
      return *this;
    }

    // At this point the state is probably empty and we have to allocate the
    // blocks
    copyFrom(other);
    return *this;
  }

  inline void copyFrom(const State& other) {
    for (BlockBase::Ptr current_block : other.state_blocks_) {
      BlockBase::Ptr new_block = current_block->clone();
      addBlock(new_block);
    }
  }

  void defineState(std::vector<BlockTypeId> block_types) {
    for (BlockTypeId current_type : block_types) {
      BlockBase::Ptr current_block = block_helper::createBlockByType(current_type);
      addBlock(current_block);
    }
  }

  void setBlock(const size_t key, const VectorXRef& value) {
    getBlock(key)->setValue(value);
  }

  template <typename BlockType>
  void setValue(const size_t key, const typename BlockType::StorageType& value) {
    BlockType* block = dynamic_cast<BlockType*>(getBlock(key).get());

    CHECK_NOTNULL(block);  // Check if cast successful
    block->setValue(value);
  }

  template <typename BlockType>
  typename BlockType::StorageType& getValue(const size_t key) {
    BlockType* block = getBlock<BlockType>(key);
    return block->template getValue<BlockType>();
  }

  inline std::vector<BlockBase::Ptr> getBlocks(const std::vector<size_t>& keys) const {
    std::vector<BlockBase::Ptr> blocks;
    for (const size_t& current_key : keys) {
      blocks.emplace_back(getBlock(current_key));
    }
    return blocks;
  }

  inline BlockBase::Ptr getBlock(const size_t key) const {
    CHECK(state_blocks_.size() > key);  // Check if key is valid.
    return state_blocks_[key];
  }

  template <typename BlockType>
  std::shared_ptr<BlockType> getBlock(const size_t key) const {
    std::shared_ptr<BlockType> block = std::dynamic_pointer_cast<BlockType>(getBlock(key));
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
      result_state->copyFrom(*this);
    }

    CHECK(dimension_ == result_state->dimension_)
        << "Dimension of first state" << dimension_ << " result state "
        << result_state
               ->dimension_;  // Check if dimension of result_state is valid.
    int accumulated_dimension = 0;

    BlockVector::iterator block_iterator = result_state->state_blocks_.begin();
    for (BlockBase::Ptr current_block : state_blocks_) {
      current_block->boxPlus(
          dx.segment(accumulated_dimension, current_block->minimal_dimension_),
          (*block_iterator).get());
      accumulated_dimension += current_block->minimal_dimension_;
      ++block_iterator;
    }
  }

  // calculates this boxminus other = dx
  void boxMinus(const State& other, VectorX* dx) const {
    CHECK_NOTNULL(dx);

    CHECK(
        minimal_dimension_ ==
        dx->size());  // Check if dimension of dx is valid.

    CHECK(dimension_ == other.dimension_)
        << "Dimension of first state " << dimension_ << " other state "
        << other.dimension_;  // Check if dimension of result_state is valid.

    int index = 0;
    BlockVector::const_iterator block_iterator = other.state_blocks_.begin();
    for (BlockBase::Ptr current_block : state_blocks_) {
      dx->segment(index, current_block->minimal_dimension_) =
          current_block->boxMinus((*block_iterator).get());
      index += current_block->minimal_dimension_;
      ++block_iterator;
    }
  }

  inline int getAccumulatedMinimalDimension(const size_t key) const {
    CHECK(key < accumulated_minimal_dimensions_.size());
    return accumulated_minimal_dimensions_[key];
  }

  VectorX getAsVector() const {
    VectorX state_vector(dimension_);
    int index = 0;
    for (BlockBase::Ptr current_block : state_blocks_) {
      state_vector.segment(index, current_block->dimension_) =
          current_block->getValueAsVector();
      index += current_block->dimension_;
    }
    return state_vector;
  }

  std::string print() const {
    std::ostringstream oss;
    for (BlockBase::Ptr current_block : state_blocks_) {
      oss << current_block->getTypeName() << "["
          << current_block->getValueAsVector().transpose() << "], ";
    }
    return oss.str();
  }

 private:
  inline void addBlock(BlockBase::Ptr block_to_add) {
//    CHECK_NOTNULL(block_to_add);
    accumulated_minimal_dimensions_.push_back(dimension_);
    dimension_ += block_to_add->dimension_;
    minimal_dimension_ += block_to_add->minimal_dimension_;
    state_blocks_.push_back(block_to_add);
  }

  BlockVector state_blocks_;
  std::vector<int> accumulated_minimal_dimensions_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_STATE_H_ */
