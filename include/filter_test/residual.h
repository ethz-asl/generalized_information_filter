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
#include "filter_test/timeline.h"
#include "filter_test/utils/logging.h"

namespace tsif {

// We assume that the residuals are embedded in a vector space (i.e. tangent
// space for manifolds).
class ResidualBase {
 public:
  const int dimension_;
  const int minimal_dimension_;  // Dimension of the tangent space
  const bool is_mergeable_;
  bool active_;

  // TODO(burrimi): Do we need the full timeline or just the
  // map<time,measurement>?
  //  std::vector<Timeline*> measurement_timelines_;

  ResidualBase(int dimension, int minimal_dimension, bool is_mergeable)
      : dimension_(dimension), minimal_dimension_(minimal_dimension), is_mergeable_(is_mergeable), active_(false) {}
  virtual ~ResidualBase() {}

  //  void setMeasurementTimelines(std::vector<Timeline*> timelines);

  //  virtual bool prepareResidual(const int64_t t1_ns, const int64_t t2_ns) = 0;

  // Predicts the state. The jacobians are only evaluated if the
  // jacobian_wrt_state1 is not null.
  // The state and jacobian blocks contain only the relevant blocks for that
  // particular residual and not the full state!
  virtual bool predict(
      const std::vector<BlockBase*>& state,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns,
      std::vector<BlockBase*>* predicted_state,
      std::vector<MatrixXRef>* jacobian_wrt_state1) = 0;

  // evaluates the residual. The jacobians are only evaluated if the
  // jacobian_wrt_state* is not null.
  // The state and jacobian blocks contain only the relevant blocks for that
  // particular residual and not the full state!
  virtual bool evaluate(
      const std::vector<BlockBase*>& state1,
      const std::vector<BlockBase*>& state2,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorXRef* residual,
      std::vector<MatrixXRef>* jacobian_wrt_state1,
      std::vector<MatrixXRef>* jacobian_wrt_state2) = 0;

  virtual std::string getName() const = 0;

  // This function checks if all input blocks are of correct type.
  virtual bool inputTypesValid(
      const std::vector<BlockBase*>& state1,
      const std::vector<BlockBase*>& state2) = 0;

//  bool finiteDifference(const std::vector<BlockBase*>& state1,
//                        const std::vector<BlockBase*>& state2,
//                        const std::vector<const TimedMeasurementVector*>& measurement_vectors,
//                        const int64_t t1_ns, const int64_t t2_ns, const double delta,
//                        std::vector<MatrixXRef>* jacobian_wrt_state1,
//                        std::vector<MatrixXRef>* jacobian_wrt_state2) {
//    VectorX residual(minimal_dimension_);
//    VectorX residual_perturbed(minimal_dimension_);
//
//    evaluate(state1, state2, measurement_vectors, t1_ns, t2_ns, &residual, nullptr, nullptr);
//
//    // Jacobians wrt state1
//    std::vector<BlockBase*> state1_perturbed;
//    for(size_t block_index = 0; block_index < state1.size(); ++block_index) {
//      const BlockBase* current_block = state1[block_index];
//      for(size_t block_offset = 0; block_offset < current_block->size(); ++block_offset) {
//        //boxplus
//        evaluate(state1_perturbed, state2, measurement_vectors, t1_ns, t2_ns, &residual_perturbed, nullptr, nullptr);
//
//        //boxminus
//        (*jacobian_wrt_state1)[block_index].col(block_offset) =  (1/delta) * (residual_perturbed-residual);
//      }
//    }
//
//    std::vector<BlockBase*> state2_perturbed;
//
//  }

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUAL_H_ */
