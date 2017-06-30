/*
 * estimator.h
 *
 *  Created on: 23.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_ESTIMATOR_H_
#define INCLUDE_FILTER_TEST_ESTIMATOR_H_

#include <iostream>
#include <list>

#include "filter_test/block.h"
#include "filter_test/defines.h"
#include "filter_test/filter.h"
#include "filter_test/helper_functions.h"
#include "filter_test/measurement_manager.h"
#include "filter_test/problem_builder.h"
#include "filter_test/residual.h"
#include "filter_test/state.h"

namespace tsif {

// Simple base function to initialize the state. Init() gets called when the first update() is possible.
// TODO(burrimi): This seems overly complex, should we switch to a callback?
class InitStateBase {
public:
  virtual ~InitStateBase() {}
  virtual bool init(const MeasurementManager& measurement_manager, const UpdateDescription& update_description, State* state, MatrixX* information) = 0;
private:
};

// Dummy state initializer just sets the information matrix to identity and does nothing to the state.
class DummyInitState: public InitStateBase {
public:
  virtual bool init(const MeasurementManager& measurement_manager, const UpdateDescription& update_description, State* state, MatrixX* information) {
    information->setIdentity();
    return true;
  }
private:
};


class Estimator {
 public:
  Estimator(InitStateBase* state_initializer) :is_initialized_(false), timestamp_previous_update_ns_(-1), state_initializer_(state_initializer) {

  }

  ~Estimator() {
    delete state_initializer_;
  }

  bool defineState(std::vector<BlockType> state_types);

  void initStateValue(const int key, const VectorXRef& value);

  // Adds a residual. The problem_builder takes ownership of this residual and takes care of cleaning it up.
  bool addResidual(ResidualBase* residual, std::vector<int> first_keys, std::vector<int> second_keys,
                   std::vector<int> measurement_keys = std::vector<int>());

  // Same as addResidual but additionally this residual is used to predict the state. This is only needed for the GIF.
  bool addPredictionResidual(ResidualBase* residual, std::vector<int> first_keys, std::vector<int> second_keys,
                   std::vector<int> measurement_keys = std::vector<int>());

  bool addResidualImplementation(ResidualBase* residual, std::vector<int> first_keys, std::vector<int> second_keys,
                                            std::vector<int> measurement_keys, const bool use_for_prediction);

  void addMeasurement(int timeline_key, int timestamp_ns, MeasurementBase* measurement);

  void printState() const;

  void printTimeline() const;

  void printResiduals() const;

  void checkResiduals() const;

 private:

  void runEstimator();

  bool init(const UpdateDescription& update_description);

  MeasurementManager measurement_manager_; // Handles all the measurments and decides when to run the filter. Currently only GIF supported.
  ProblemBuilder problem_builder_; // Handles the residuals and assembles the FilterProblemDescription which is solved by the filter.
  Filter filter_; // Filtering algorithm we use. Currently only GIF implemented and hardcoded.

  InitStateBase* state_initializer_;

  bool is_initialized_;

  // defines the state.
  // TODO(burrimi): State could also automatically be assembled by the residuals in the future.
  std::vector<BlockType> state_types_;

  // Most recent state and corresponding information (= inverse of covariance matrix).
  // TODO(burrimi): Replace this with a state buffer, to allow updates in the past.
  State state_;
  MatrixX information_;
  int timestamp_previous_update_ns_;

};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_FILTER_H_ */
