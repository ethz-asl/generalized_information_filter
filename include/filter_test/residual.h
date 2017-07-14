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
  const int dimension_; // TODO(burrimi): do we need this? remove otherwise!
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
      const VectorOfBlocks& state,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns,
      VectorOfBlocks* predicted_state,
      std::vector<MatrixXRef>* jacobian_wrt_state1) = 0;

  // evaluates the residual. The jacobians are only evaluated if the
  // jacobian_wrt_state* is not null.
  // The state and jacobian blocks contain only the relevant blocks for that
  // particular residual and not the full state!
  // Unfortunately passing residual as a pointer did not work. Maybe related to https://stackoverflow.com/questions/35612831/eigenref-in-pass-by-pointer
  virtual bool evaluate(
      const VectorOfBlocks& state1,
      const VectorOfBlocks& state2,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorXRef residual,
      std::vector<MatrixXRef>* jacobian_wrt_state1,
      std::vector<MatrixXRef>* jacobian_wrt_state2) = 0;

  virtual std::string getName() const = 0;

  // This function checks if all input blocks are of correct type.
  virtual bool inputTypesValid(
      const VectorOfBlocks& state1,
      const VectorOfBlocks& state2) = 0;

  // This function checks if all jacobians are implemented correctly.
  // In case you use finite differences for your residual jacobians this function is obviously not required.
  virtual bool checkJacobians(const VectorOfBlocks& state1,
                        const VectorOfBlocks& state2,const int64_t t1_ns, const int64_t t2_ns, const double delta) {
    return true;
  }



 protected:
  void finiteDifference(const VectorOfBlocks& state1,
                        const VectorOfBlocks& state2,
                        const std::vector<const TimedMeasurementVector*>& measurement_vectors,
                        const int64_t t1_ns, const int64_t t2_ns, const double delta,
                        std::vector<MatrixXRef>* jacobian_wrt_state1,
                        std::vector<MatrixXRef>* jacobian_wrt_state2);

  bool checkJacobiansImpl(const VectorOfBlocks& state1,
                         const VectorOfBlocks& state2,
                         const std::vector<const TimedMeasurementVector*>& measurement_vectors,
                         const int64_t t1_ns, const int64_t t2_ns, const double delta) {

     if(state1.empty() && state2.empty()) {
       return true;
     }

     const int state1_minimal_dimension = block_helper::getMinimalDimension(state1);
     const int state2_minimal_dimension = block_helper::getMinimalDimension(state2);
     MatrixX J1(minimal_dimension_, state1_minimal_dimension);
     MatrixX J2(minimal_dimension_, state2_minimal_dimension);
     std::vector<MatrixXRef> J1_blocks = getJacobianBlocks(state1, &J1);
     std::vector<MatrixXRef> J2_blocks = getJacobianBlocks(state2, &J2);

     MatrixX J1_finite_diff(minimal_dimension_, state1_minimal_dimension);
     MatrixX J2_finite_diff(minimal_dimension_, state2_minimal_dimension);
     std::vector<MatrixXRef> J1_blocks_finite_diff = getJacobianBlocks(state1, &J1_finite_diff);
     std::vector<MatrixXRef> J2_blocks_finite_diff = getJacobianBlocks(state2, &J2_finite_diff);

     VectorX residual(minimal_dimension_);
     evaluate(state1, state2, measurement_vectors, t1_ns, t2_ns, residual, nullptr, nullptr);
     evaluate(state1, state2, measurement_vectors, t1_ns, t2_ns, residual, &J1_blocks, &J2_blocks);
     finiteDifference(state1, state2, measurement_vectors, t1_ns, t2_ns, delta, &J1_blocks_finite_diff, &J2_blocks_finite_diff);

     bool all_ok = true;
     if(!J1.isApprox(J1_finite_diff, 1e-6)) {
       TSIF_LOGE("Residual Jacobian might be wrong: \n" << J1 << "\n Num diff: \n" << J1_finite_diff);
       all_ok = false;
     }

     if(!J2.isApprox(J2_finite_diff, 1e-6)) {
       TSIF_LOGE("Residual Jacobian might be wrong: \n" << J2 << "\n Num diff: \n" << J2_finite_diff);
       all_ok = false;
     }
     return all_ok;
    }

 private:
  std::vector<MatrixXRef> getJacobianBlocks(
      const VectorOfBlocks& state, MatrixX* jacobian) {
    CHECK_NOTNULL(jacobian);
    std::vector<MatrixXRef> jacobian_blocks;

    int state_index = 0;
    for (size_t i = 0; i < state.size(); ++i) {
      jacobian_blocks.emplace_back(jacobian->block(
          0, state_index, minimal_dimension_,
          state[i]->minimal_dimension_));
      state_index += state[i]->minimal_dimension_;
    }
    return jacobian_blocks;
  }

};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUAL_H_ */
