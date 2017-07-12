/*
 * filter.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_FILTER_H_
#define INCLUDE_FILTER_TEST_FILTER_H_

#include <iostream>
#include <list>

#include "filter_test/block.h"
#include "filter_test/defines.h"
#include "filter_test/helper_functions.h"
#include "filter_test/measurement.h"
#include "filter_test/problem_builder.h"  // TODO(burrimi):remove this dependency. ResidualContainer
#include "filter_test/state.h"

namespace tsif {

// TODO(burrimi): Buffer the prepared residuals.
// Each residual only depends on a couple of states and parts of the jacobian.
// Now in each iteration we extract these blocks again and put them into a
// vector which is a lot of overhead and we should buffer this.
// struct PreparedResidual {
//  std::vector<BlockBase::Ptr> blocks1;
//  std::vector<BlockBase::Ptr> blocks2;
//  VectorXRef residual;
//  std::vector<MatrixXRef> jacobian_wrt_state1_blocks;
//  std::vector<MatrixXRef> jacobian_wrt_state2_blocks;
//};

struct FilterConfig {
  int max_update_iterations = 10;
  double residual_norm_threshold = 0.1;
};

class Filter {
 public:
  Filter() {}

  ~Filter() {}

  void setConfig(const FilterConfig& config) {
    config_ = config;
  }

  void predictState(
      const MeasurementBuffer& measurement_buffer,
      const FilterProblemDescription& filter_problem, const State& state,
      State* predicted_state) const;

  void constructProblem(
      const MeasurementBuffer& measurement_buffer,
      const FilterProblemDescription& filter_problem, const State& first_state,
      const State& second_state, VectorX* residual_vector,
      MatrixX* jacobian_wrt_state1, MatrixX* jacobian_wrt_state2);

  bool init(const State& state, const int total_residual_dimension);

  void predictAndUpdate(
      const MeasurementBuffer& measurement_buffer,
      const FilterProblemDescription& filter_problem, const State& state,
      const MatrixX& information, State* updated_state,
      MatrixX* updated_information);

 private:
  inline std::vector<MatrixXRef> getJacobianBlocks(
      const State& state, const std::vector<size_t>& keys,
      const int residual_index, const int residual_dimension,
      MatrixX* jacobian) {
    std::vector<MatrixXRef> jacobian_blocks;
    for (const size_t& current_key : keys) {
      const int& state_index =
          state.getAccumulatedMinimalDimension(current_key);
      jacobian_blocks.emplace_back(jacobian->block(
          residual_index, state_index, residual_dimension,
          state.minimal_dimension_));
    }
    return jacobian_blocks;
  }

  void setMatrixDimensions(
      const int active_residuals_dimension, const int minimal_state_dimension);

  FilterConfig config_;

  // Temporary variables
  State temporary_second_state_;
  VectorX residual_vector_;
  MatrixX jacobian_wrt_state1_;
  MatrixX jacobian_wrt_state2_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_FILTER_H_ */
