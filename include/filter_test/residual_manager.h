/*
 * residual_manager.h
 *
 *  Created on: 01.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_RESIDUAL_MANAGER_H_
#define INCLUDE_FILTER_TEST_RESIDUAL_MANAGER_H_

#include "filter_test/residual.h"

namespace tsif {

struct ResidualContainer {
  std::vector<int> first_keys;
  std::vector<int> second_keys;
  ResidualBase* residual;
};

class ResidualManager {
 public:
  ResidualManager() : dimension_(0) {}

  ~ResidualManager() {
    for (ResidualContainer& current_residual : residuals_) {
      delete current_residual.residual;
    }
  }

  // This function adds a residual and takes ownership of the residual.
  bool addResidual(ResidualBase* residual, std::vector<std::string> in, std::vector<std::string> out);

  bool checkAllResidualTypes();

 private:
  std::list<ResidualContainer> residuals_;
  int dimension_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUAL_MANAGER_H_ */
