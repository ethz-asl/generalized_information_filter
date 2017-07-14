/*
 * block.h
 *
 *  Created on: 01.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_BLOCK_H_
#define INCLUDE_FILTER_TEST_BLOCK_H_

#include <iostream>
#include <vector>

#include <glog/logging.h>

#include "filter_test/defines.h"

namespace tsif {

class BlockBase;

using BlockBasePtr = std::shared_ptr<BlockBase>;
using VectorOfBlocks = std::vector<std::shared_ptr<BlockBase>>;

enum BlockTypeId {
  kVector1 = 0,
  kVector2,
  kVector3,
  kVector4,
  kVector5,
  kVector6,
  kSO1,         // Not implemented
  kSO2,         // Not implemented
  kSO3,         // Not implemented
  kUnitVector3  // Not implemented
};

class BlockBase {
 public:
  typedef BlockBasePtr Ptr;
  const int dimension_;
  const int minimal_dimension_;  // Dimension of the tangent space
  const bool is_vector_space_;
  BlockBase(int dimension, int minimal_dimension, bool is_vector_space)
      : dimension_(dimension),
        minimal_dimension_(minimal_dimension),
        is_vector_space_(is_vector_space) {
    CHECK(dimension >= minimal_dimension);
  };
  virtual ~BlockBase() {}

  virtual BlockBase::Ptr clone() const = 0;

  virtual void copyBlockTo(
      BlockBase* result) const = 0;  // TODO(burrimi): find better way.

  template <typename BlockType>
  typename BlockType::StorageType& getValue() {
    BlockType* block = dynamic_cast<BlockType*>(this);
    CHECK_NOTNULL(block);  // // Check if cast successful
    return block->getValue();
  }

  template <typename BlockType>
  const typename BlockType::StorageType& getValue() const {
    const BlockType* block = dynamic_cast<const BlockType*>(this);
    CHECK_NOTNULL(block);  // // Check if cast successful
    return block->getValue();
  }

  template <typename BlockType>
  bool isBlockTypeCorrect() const {
    const BlockType* block = dynamic_cast<const BlockType*>(this);
    if (block == nullptr) {
      return false;
    }
    return true;
  }

  virtual void boxPlus(const Eigen::VectorXd& dx, BlockBase* result) = 0;
  virtual Eigen::VectorXd boxMinus(const BlockBase* y) = 0;
  virtual Eigen::VectorXd getValueAsVector() = 0;
  virtual void setValueFromVector(const VectorXRef& value) = 0;
  virtual std::string getTypeName() = 0;
  virtual void setRandom() = 0;

 private:
};


namespace block_helper {
BlockBase::Ptr createBlockByType(BlockTypeId block_type);

inline void copyVectorOfBlocks(const VectorOfBlocks& a, VectorOfBlocks* b) {
  CHECK_NOTNULL(b);
  b->resize(a.size());
  for (size_t i = 0; i < a.size(); ++i) {
    (*b)[i] = a[i]->clone();
  }
}

inline int getMinimalDimension(const VectorOfBlocks& a) {
  int accumulated_dimension = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    accumulated_dimension += a[i]->minimal_dimension_;
  }
  return accumulated_dimension;
}

inline void setRandom(VectorOfBlocks* blocks) {
  CHECK_NOTNULL(blocks);
  for (size_t i = 0; i < blocks->size(); ++i) {
    (*blocks)[i]->setRandom();
  }
}

// calculates a boxplus dx = b
inline void boxPlus(
    const VectorOfBlocks& a, const int minimal_dimension_a,
    const VectorXRef& dx, VectorOfBlocks* b) {
  CHECK_NOTNULL(b);
  CHECK(minimal_dimension_a == dx.size());
  CHECK(a.size() == b->size());

  int accumulated_dimension = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    a[i]->boxPlus(
        dx.segment(accumulated_dimension, a[i]->minimal_dimension_),
        (*b)[i].get());
    accumulated_dimension += a[i]->minimal_dimension_;
  }
}

inline void boxPlus(
    const VectorOfBlocks& a, const VectorXRef& dx, VectorOfBlocks* b) {
  boxPlus(a, getMinimalDimension(a), dx, b);
}

// calculates a boxminus b = dx
inline void boxMinus(
    const VectorOfBlocks& a, const int minimal_dimension_a,
    const VectorOfBlocks& b, VectorX* dx) {
  CHECK_NOTNULL(dx);
  CHECK(minimal_dimension_a == dx->size());
  CHECK(a.size() == b.size());

  int accumulated_dimension = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    dx->segment(accumulated_dimension, a[i]->minimal_dimension_) =
        a[i]->boxMinus(b[i].get());
    accumulated_dimension += a[i]->minimal_dimension_;
  }
}

inline void boxMinus(
    const VectorOfBlocks& a, const VectorOfBlocks& b, VectorX* dx) {
  boxMinus(a, getMinimalDimension(a), b, dx);
}

}  // namespace block_helper

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_BLOCK_H_ */
