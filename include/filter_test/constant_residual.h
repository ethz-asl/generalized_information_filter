/*
 * constant_residual.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_CONSTANT_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_CONSTANT_RESIDUAL_H_

#include "filter_test/residual.h"

// We assume that the residuals are embedded in a vector space (i.e. tangent space for manifolds).
class ConstantResidual: public ResidualBase {
  static const bool kIsMergeable = true;
  static const int kResidualDimension = 3;
public:
  ConstantResidual(): ResidualBase(kResidualDimension, kIsMergeable) {
    state1_block_types_.push_back(BlockType::kVector3);
    state2_block_types_.push_back(BlockType::kVector3);
  }

  ~ConstantResidual() {}

  virtual bool evaluate(const std::vector<BlockBase*>& state1,
                        const std::vector<BlockBase*>& state2,
                        const int t1_ns, const int t2_ns,
                        VectorXRef* residual, std::vector<MatrixXRef>* jacobian_wrt_state1,
                        std::vector<MatrixXRef>* jacobian_wrt_state2) {return true;}

  virtual std::string getResidualName() {return "const residual";}

private:
};



#endif /* INCLUDE_FILTER_TEST_CONSTANT_RESIDUAL_H_ */
