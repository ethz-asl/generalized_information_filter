/*
 * residual.cpp
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#include "filter_test/residual.h"

namespace tsif {

// void ResidualBase::setMeasurementTimelines(std::vector<Timeline*> timelines)
// { measurement_timelines_ = timelines; }

// TODO(burrimi): Clean up this function!!!
void ResidualBase::finiteDifference(
    const VectorOfBlocks& state1, const VectorOfBlocks& state2,
    const std::vector<const TimedMeasurementVector*>& measurement_vectors,
    const int64_t t1_ns, const int64_t t2_ns, const double delta,
    std::vector<MatrixXRef>* jacobian_wrt_state1,
    std::vector<MatrixXRef>* jacobian_wrt_state2) {
  VectorX residual(minimal_dimension_);
  VectorX residual_perturbed(minimal_dimension_);
  const int state1_minimal_dimension =
      block_helper::getMinimalDimension(state1);
  const int state2_minimal_dimension =
      block_helper::getMinimalDimension(state2);

  evaluate(
      state1, state2, measurement_vectors, t1_ns, t2_ns, residual, nullptr,
      nullptr);

  // Jacobians wrt state1
  if (jacobian_wrt_state1 != nullptr) {
    VectorOfBlocks state1_perturbed;
    block_helper::copyVectorOfBlocks(state1, &state1_perturbed);

    for (size_t block_index = 0; block_index < state1.size(); ++block_index) {
      const BlockBase::Ptr& current_block = state1[block_index];
      VectorX delta_vector(current_block->minimal_dimension_);

      for (int block_offset = 0;
           block_offset < current_block->minimal_dimension_; ++block_offset) {
        delta_vector.setZero();
        delta_vector[block_offset] = delta;
        // boxplus of relevant block
        state1[block_index]->boxPlus(
            delta_vector, state1_perturbed[block_index].get());

        evaluate(
            state1_perturbed, state2, measurement_vectors, t1_ns, t2_ns,
            residual_perturbed, nullptr, nullptr);

        (*jacobian_wrt_state1)[block_index].col(block_offset) =
            (1 / delta) * (residual_perturbed - residual);
      }
      state1_perturbed[block_index] =
          state1[block_index];  // TODO(burrimi):remove hack!
    }
  }

  if (jacobian_wrt_state2 != nullptr) {
    VectorOfBlocks state2_perturbed;
    block_helper::copyVectorOfBlocks(state2, &state2_perturbed);

    for (size_t block_index = 0; block_index < state2.size(); ++block_index) {
      const BlockBase::Ptr& current_block = state2[block_index];
      VectorX delta_vector(current_block->minimal_dimension_);

      for (int block_offset = 0;
           block_offset < current_block->minimal_dimension_; ++block_offset) {
        delta_vector.setZero();
        delta_vector[block_offset] = delta;
        // boxplus of relevant block
        state2[block_index]->boxPlus(
            delta_vector, state2_perturbed[block_index].get());

        evaluate(
            state1, state2_perturbed, measurement_vectors, t1_ns, t2_ns,
            residual_perturbed, nullptr, nullptr);

        (*jacobian_wrt_state2)[block_index].col(block_offset) =
            (1 / delta) * (residual_perturbed - residual);
      }
      state2_perturbed[block_index] =
          state2[block_index];  // TODO(burrimi):remove hack!
    }
  }
}

}  // namespace tsif
