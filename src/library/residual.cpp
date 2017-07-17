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

bool ResidualBase::checkJacobiansImpl(
     const VectorOfBlocks& state1, const VectorOfBlocks& state2,
     const std::vector<const TimedMeasurementVector*>& measurement_vectors,
     const int64_t t1_ns, const int64_t t2_ns, const double delta) {
   if (state1.empty() && state2.empty()) {
     return true;
   }

   const int state1_minimal_dimension =
       block_helper::getMinimalDimension(state1);
   const int state2_minimal_dimension =
       block_helper::getMinimalDimension(state2);
   MatrixX J1(minimal_dimension_, state1_minimal_dimension);
   MatrixX J2(minimal_dimension_, state2_minimal_dimension);
   std::vector<MatrixXRef> J1_blocks = getJacobianBlocks(state1, &J1);
   std::vector<MatrixXRef> J2_blocks = getJacobianBlocks(state2, &J2);

   MatrixX J1_finite_diff(minimal_dimension_, state1_minimal_dimension);
   MatrixX J2_finite_diff(minimal_dimension_, state2_minimal_dimension);
   std::vector<MatrixXRef> J1_blocks_finite_diff =
       getJacobianBlocks(state1, &J1_finite_diff);
   std::vector<MatrixXRef> J2_blocks_finite_diff =
       getJacobianBlocks(state2, &J2_finite_diff);

   VectorX residual(minimal_dimension_);
   evaluate(
       state1, state2, measurement_vectors, t1_ns, t2_ns, residual, nullptr,
       nullptr);
   evaluate(
       state1, state2, measurement_vectors, t1_ns, t2_ns, residual, &J1_blocks,
       &J2_blocks);
   finiteDifference(
       state1, state2, measurement_vectors, t1_ns, t2_ns, delta,
       &J1_blocks_finite_diff, &J2_blocks_finite_diff);

   bool all_ok = true;
   if (!J1.isApprox(J1_finite_diff, 1e-6)) {
     TSIF_LOGE(
         "Residual Jacobian might be wrong: \n"
         << J1 << "\n Num diff: \n"
         << J1_finite_diff);
     all_ok = false;
   }

   if (!J2.isApprox(J2_finite_diff, 1e-6)) {
     TSIF_LOGE(
         "Residual Jacobian might be wrong: \n"
         << J2 << "\n Num diff: \n"
         << J2_finite_diff);
     all_ok = false;
   }
   return all_ok;
 }

void ResidualBase::finiteDifference(
    const VectorOfBlocks& state1, const VectorOfBlocks& state2,
    const std::vector<const TimedMeasurementVector*>& measurement_vectors,
    const int64_t t1_ns, const int64_t t2_ns, const double delta,
    std::vector<MatrixXRef>* jacobian_wrt_state1,
    std::vector<MatrixXRef>* jacobian_wrt_state2) {
  VectorX residual(minimal_dimension_);
  VectorX residual_perturbed(minimal_dimension_);
  CHECK(delta > 0.0); // Finite difference with 0 does not make sense..

  evaluate(
      state1, state2, measurement_vectors, t1_ns, t2_ns, residual, nullptr,
      nullptr);

  // Jacobians wrt state1
  if (jacobian_wrt_state1 != nullptr) {
    const int state1_minimal_dimension =
        block_helper::getMinimalDimension(state1);
    VectorOfBlocks state1_perturbed;
    block_helper::copyVectorOfBlocks(state1, &state1_perturbed);

    VectorX delta_vector(state1_minimal_dimension);
    delta_vector.setZero();
    int index_to_test = 0;

    for (size_t block_index = 0; block_index < state1.size(); ++block_index) {
      const BlockBase::Ptr& current_block = state1[block_index];

      for (int block_offset = 0;
           block_offset < current_block->minimal_dimension_; ++block_offset) {
        delta_vector[index_to_test] = delta;
        // boxplus of relevant block
        block_helper::boxPlus(
            state1, state1_minimal_dimension, delta_vector, &state1_perturbed);
        evaluate(
            state1_perturbed, state2, measurement_vectors, t1_ns, t2_ns,
            residual_perturbed, nullptr, nullptr);

        (*jacobian_wrt_state1)[block_index].col(block_offset) =
            (1 / delta) * (residual_perturbed - residual);
        delta_vector[index_to_test] = 0;  // reset delta vector
        ++index_to_test;
      }
    }
  }

  if (jacobian_wrt_state2 != nullptr) {
    const int state2_minimal_dimension =
        block_helper::getMinimalDimension(state2);
    VectorOfBlocks state2_perturbed;
    block_helper::copyVectorOfBlocks(state2, &state2_perturbed);

    VectorX delta_vector(state2_minimal_dimension);
    delta_vector.setZero();
    int index_to_test = 0;

    for (size_t block_index = 0; block_index < state2.size(); ++block_index) {
      const BlockBase::Ptr& current_block = state2[block_index];

      for (int block_offset = 0;
           block_offset < current_block->minimal_dimension_; ++block_offset) {
        delta_vector[index_to_test] = delta;
        // boxplus of relevant block
        block_helper::boxPlus(
            state2, state2_minimal_dimension, delta_vector, &state2_perturbed);

        evaluate(
            state1, state2_perturbed, measurement_vectors, t1_ns, t2_ns,
            residual_perturbed, nullptr, nullptr);

        (*jacobian_wrt_state2)[block_index].col(block_offset) =
            (1 / delta) * (residual_perturbed - residual);

        delta_vector[index_to_test] = 0;  // reset delta vector
        ++index_to_test;
      }
    }
  }
}

}  // namespace tsif
