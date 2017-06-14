/*
 * block.h
 *
 *  Created on: 01.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_BLOCK_H_
#define INCLUDE_FILTER_TEST_BLOCK_H_

#include <iostream>

#include <glog/logging.h>

#include "filter_test/block.h"
#include "filter_test/defines.h"

enum BlockType {
  kVector1 = 0, // Vector types need to be in correct order, otherwise type checks won't work!
  kVector2,
  kVector3,
  kVector4,
  kVector5,
  kVector6,
  kSO3
};


class BlockBase {
 public:
  const int minimal_dimension_; // Dimension of the tangent space
  const int dimension_;
  const BlockType type_;
  BlockBase(int o_dim, int dimension, BlockType type): minimal_dimension_(dimension),
      dimension_(o_dim), type_(type) {
    CHECK(o_dim >= dimension);
  };
  virtual ~BlockBase() {}

  virtual BlockBase* clone() const = 0;

  virtual void copyBlockTo(BlockBase* result) const = 0; // TODO(burrimi): find better way.


  virtual void boxPlus(const Eigen::VectorXd& dx, BlockBase* result) = 0;
  virtual Eigen::VectorXd boxMinus(const BlockBase* y) = 0;
  virtual Eigen::VectorXd getValue() = 0;
  virtual void setValue(const VectorXRef& value) = 0;
  virtual std::string getTypeName() = 0;
 private:
};


//template <class Derived>
//class BlockHelper : public BlockBase
//{
//public:
//  virtual BlockBase* clone() const
//  {
//    return new Derived(static_cast<const Derived&>(*this)); // call the copy ctor.
//  }
//};


template <int Dimension>
class VectorBlock: public BlockBase {
 public:
  // TODO(burrimi): remove ugly hack for type.
  VectorBlock(const Vector<Dimension>& value):BlockBase(Dimension, Dimension,
                                                        static_cast<BlockType>(Dimension + BlockType::kVector1 - 1)),
                                                        value_(value){ }

  VectorBlock():VectorBlock(Vector<Dimension>::Zero()) { }

  virtual ~VectorBlock() {}

  virtual BlockBase* clone() const
  {
    return new VectorBlock<Dimension>(static_cast<const VectorBlock<Dimension>&>(*this)); // call the copy ctor.
  }

  virtual void copyBlockTo(BlockBase* result) const {
    VectorBlock<Dimension>* result1 = dynamic_cast<VectorBlock<Dimension>* >(result);
    CHECK_NOTNULL(result1);
    result1->value_ = value_;
  }

  virtual void boxPlus(const Eigen::VectorXd& dx, BlockBase* result) {
    CHECK(dx.size() == Dimension);
    VectorBlock<Dimension>* result1 = dynamic_cast<VectorBlock<Dimension>* >(result);
    CHECK_NOTNULL(result1);
    result1->value_ = value_ + dx;
  }

  virtual Eigen::VectorXd boxMinus(const BlockBase* y) {
    const VectorBlock<Dimension>* y_vector = dynamic_cast<const VectorBlock<Dimension>* >(y);
    CHECK_NOTNULL(y_vector);// << "Type cast from BlockBase to VectorBlock failed! Did you mix types?";
    Vector<Dimension> result = value_ - y_vector->value_;
    return result;
  }

  virtual Eigen::VectorXd getValue() {return value_;}

  virtual void setValue(const VectorXRef& value) {
    CHECK(value.size() == Dimension);
    value_ = value;
  }

  virtual std::string getTypeName() {return "vector" + std::to_string(Dimension);}

  Vector<Dimension> value_;
 private:
};

namespace block_helper {
BlockBase* createBlockByType(BlockType block_type);
}  // namespace block_helper

#endif /* INCLUDE_FILTER_TEST_BLOCK_H_ */
