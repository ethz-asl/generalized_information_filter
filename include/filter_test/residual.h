/*
 * residual.h
 *
 *  Created on: 01.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_RESIDUAL_H_

#include <vector>

#include "filter_test/block.h"

// We assume that the residuals are embedded in a vector space (i.e. tangent space for manifolds).
class ResidualBase {
 public:
  const int dimension_;

  std::vector<BlockType> state1_block_types_;
  std::vector<BlockType> state2_block_types_;

  ResidualBase(int dimension):dimension_(dimension) {}
  virtual ~ResidualBase() {}

  bool inputTypesValid(const std::vector<BlockBase*>& state1,
                       const std::vector<BlockBase*>& state2); // Do some sanity checks if all types match

  virtual bool evaluate(const std::vector<BlockBase*>& state1,
                        const std::vector<BlockBase*>& state2,
                        const double t1, const double t2,
                        VectorXRef* residual, MatrixXRef* jacobian_wrt_state1,
                        MatrixXRef* jacobian_wrt_state2) = 0;
  virtual std::string getResidualName() = 0;
 private:
};



#endif /* INCLUDE_FILTER_TEST_RESIDUAL_H_ */
