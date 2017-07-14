/*
 * block_impl.h
 *
 *  Created on: 14.07.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_BLOCK_IMPL_H_
#define INCLUDE_FILTER_TEST_BLOCK_IMPL_H_


#include <glog/logging.h>

#include "filter_test/block.h"
#include "filter_test/utils/geometry.h"
#include "filter_test/utils/random.h"

namespace tsif {

template <int Dimension>
class VectorBlock : public BlockBase {
 public:
  typedef Vector<Dimension> StorageType;

  static const bool kIsVectorSpace = true;
  VectorBlock(const StorageType& value)
      : BlockBase(Dimension, Dimension, kIsVectorSpace), value_(value) {}

  VectorBlock() : VectorBlock(StorageType::Zero()) {}

  virtual ~VectorBlock() {}

  virtual BlockBase::Ptr clone() const {
    return std::make_shared<VectorBlock<Dimension>>(
        static_cast<const VectorBlock<Dimension>&>(
            *this));  // call the copy ctor.
  }

  virtual void copyBlockTo(BlockBase* result) const {
    VectorBlock<Dimension>* result1 =
        dynamic_cast<VectorBlock<Dimension>*>(result);
    CHECK_NOTNULL(result1);
    result1->value_ = value_;
  }

  virtual void boxPlus(const Eigen::VectorXd& dx, BlockBase* result) {
    CHECK(dx.size() == Dimension);
    VectorBlock<Dimension>* result1 =
        dynamic_cast<VectorBlock<Dimension>*>(result);
    CHECK_NOTNULL(result1);
    result1->value_ = value_ + dx;
  }

  virtual Eigen::VectorXd boxMinus(const BlockBase* y) {
    const VectorBlock<Dimension>* y_vector =
        dynamic_cast<const VectorBlock<Dimension>*>(y);
    CHECK_NOTNULL(y_vector);  // << "Type cast from BlockBase to VectorBlock
                              // failed! Did you mix types?";
    StorageType result = value_ - y_vector->value_;
    return result;
  }

  virtual Eigen::VectorXd getValueAsVector() {
    return value_;
  }  // TODO(burrimi): return reference?

  virtual void setValueFromVector(const VectorXRef& value) {
    CHECK(value.size() == Dimension);
    value_ = value;
  }

  void setValue(const StorageType& value) {
    value_ = value;
  }

  StorageType& getValue() {
    return value_;
  }

  const StorageType& getValue() const {
    return value_;
  }

  virtual std::string getTypeName() {
    return "vector" + std::to_string(Dimension);
  }

  virtual void setRandom() {
    value_ = NormalRandomNumberGenerator::getInstance()
                 .template getVector<Dimension>();
  }

 private:
  StorageType value_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_BLOCK_IMPL_H_ */
