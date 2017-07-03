/*
 * problem_builder.h
 *
 *  Created on: 26.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_PROBLEM_BUILDER_H_
#define INCLUDE_FILTER_TEST_PROBLEM_BUILDER_H_

#include "filter_test/measurement_manager.h"
#include "filter_test/residual.h"
#include "filter_test/state.h"

namespace tsif {

struct ResidualContainer {
  std::vector<int> first_keys;
  std::vector<int> second_keys;
  std::vector<int> measurement_keys;
  bool use_for_prediction;
  ResidualBase* residual;
};

struct FilterProblemDescription {
  std::vector<ResidualContainer*> prediction_residuals_;
  std::vector<ResidualContainer*> update_residuals_;
  int residuals_dimension_;
};

class ProblemBuilder {
 public:
  ProblemBuilder() : total_residual_dimension_(0) {}
  ~ProblemBuilder() {
    for (ResidualContainer& current_residual : residual_containers_) {
      delete current_residual.residual;
    }
  }
  // everything related to the residuals
  std::vector<ResidualContainer> residual_containers_;

  int total_residual_dimension_;

  // Adds a residual and takes ownership of the residual.
  bool addResidual(
      ResidualBase* residual, std::vector<int> first_keys,
      std::vector<int> second_keys, std::vector<int> measurement_keys,
      bool use_for_prediction);

  void printResiduals(const State& state) const;

  void checkResiduals(const State& state) const;

  int getTotalResidualDimension() const {
    return total_residual_dimension_;
  }

  FilterProblemDescription getFilterProblemDescription(
      const MeasurementBuffer& measurement_buffer);

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_PROBLEM_BUILDER_H_ */
