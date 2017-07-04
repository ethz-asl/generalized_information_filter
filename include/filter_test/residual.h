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
  const bool is_mergeable_;
  bool active_;

  // TODO(burrimi): Do we need the full timeline or just the
  // map<time,measurement>?
  //  std::vector<Timeline*> measurement_timelines_;

  ResidualBase(int dimension, bool is_mergeable)
      : dimension_(dimension), is_mergeable_(is_mergeable), active_(false) {}
  virtual ~ResidualBase() {}

  bool inputTypesValid(
      const std::vector<BlockBase*>& state1,
      const std::vector<BlockBase*>& state2)
      const;  // Do some sanity checks if all types match

  //  void setMeasurementTimelines(std::vector<Timeline*> timelines);

  //  virtual bool prepareResidual(const int t1_ns, const int t2_ns) = 0;

  virtual bool predict(
      const std::vector<BlockBase*>& state,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int t1_ns, const int t2_ns,
      std::vector<BlockBase*>* predicted_state,
      std::vector<MatrixXRef>* jacobian_wrt_state1) = 0;

  virtual bool evaluate(
      const std::vector<BlockBase*>& state1,
      const std::vector<BlockBase*>& state2,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int t1_ns, const int t2_ns, VectorXRef* residual,
      std::vector<MatrixXRef>* jacobian_wrt_state1,
      std::vector<MatrixXRef>* jacobian_wrt_state2) = 0;

  virtual std::string getPrintableName() const = 0;

  // This function checks if all input blocks are of correct type.
  virtual bool inputTypesValid(
      const std::vector<BlockBase*>& state1,
      const std::vector<BlockBase*>& state2) = 0;

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUAL_H_ */
