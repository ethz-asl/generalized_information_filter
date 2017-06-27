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


class Estimator {
 public:
  Estimator() :first_run_(true), timestamp_previous_update_ns_(-1) {

  }

  ~Estimator() {}

  // state related stuff
  std::vector<BlockType> state_types_;
  // std::vector<std::string> state_names_;

  State first_state_;
  MatrixX information_;


  ProblemBuilder problem_builder_;

  MeasurementManager measurement_manager_;

  Filter filter_;


  bool defineState(std::vector<BlockType> state_types);

  void initStateValue(const int key, const VectorXRef& value);

  // Adds a residual. The problem_builder takes ownership of this residual and takes care of cleaning it up.
  bool addResidual(ResidualBase* residual, std::vector<int> first_keys, std::vector<int> second_keys,
                   std::vector<int> measurement_keys = std::vector<int>());

  void addMeasurement(int timeline_key, int timestamp_ns, MeasurementBase* measurement);


  void printState() const;

  void printTimeline() const;

  void printResiduals() const;

  void checkResiduals();

 private:

  void runEstimator();

  bool init();

  bool first_run_;
  int timestamp_previous_update_ns_;

};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_FILTER_H_ */
