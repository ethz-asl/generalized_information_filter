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
#include "filter_test/measurement_manager.h"
#include "filter_test/problem_builder.h" // TODO(burrimi):remove this dependency. ResidualContainer
#include "filter_test/state.h"

namespace tsif {



struct PreparedResidual {
  std::vector<BlockBase*> blocks1;
  std::vector<BlockBase*> blocks2;
  VectorXRef residual;
  std::vector<MatrixXRef> jacobian_wrt_state1_blocks;
  std::vector<MatrixXRef> jacobian_wrt_state2_blocks;
};

struct FilterConfig {
  FilterConfig(): max_update_iterations(10), residual_norm_threshold(0.1){}
  int max_update_iterations;
  double residual_norm_threshold;
};

class Filter {
 public:
  Filter() : total_residual_dimension_(0) { }

  ~Filter() {
    for (ResidualContainer& current_residual : residual_containers_) {
      delete current_residual.residual;
    }
  }

  void setConfig(const FilterConfig& config) {config_ = config;}

  // state related stuff
  std::vector<BlockType> state_types_;
  // std::vector<std::string> state_names_;

  State temporary_second_state_;

  MatrixX information_;
  VectorX residual_vector_;
  MatrixX jacobian_wrt_state1_;
  MatrixX jacobian_wrt_state2_;

  // everything related to the residuals
  std::vector<ResidualContainer> residual_containers_;
  std::vector<int> prediction_residual_ids_;

  int total_residual_dimension_;

  void predictState(const FilterProblemDescription& filter_problem, const State& state, const int timestamp_previous_update_ns, const int timestamp_ns, State* predicted_state) const;
  void constructProblem(const FilterProblemDescription& filter_problem, const State& first_state, const State& second_state, VectorX* residual_vector, MatrixX* jacobian_wrt_state1, MatrixX* jacobian_wrt_state2);

  bool init(const State& state, const int& total_residual_dimension);

  void predictAndUpdate(const UpdateDescription& update_description, const FilterProblemDescription& filter_problem, const State& state, State* updated_state);


 private:

  inline std::vector<MatrixXRef> getJacobianBlocks(const State& state, const std::vector<int>& keys,
                                                   const int& residual_index, const int& residual_dimension, MatrixX* jacobian) {
    std::vector<MatrixXRef> jacobian_blocks;
    for (const int& current_key : keys) {
      //    MatrixXRef test = jacobian.block(residual_index, current_key, residual_dimension,
      //    first_state_.minimal_dimension_);
      const int& state_index = state.getAccumulatedMinimalDimension(current_key);
      jacobian_blocks.emplace_back(
          jacobian->block(residual_index, state_index, residual_dimension, state.minimal_dimension_));
    }
    return jacobian_blocks;
  }


  void setMatrixDimensions(const int active_residuals_dimension, const int minimal_state_dimension);


  FilterConfig config_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_FILTER_H_ */
