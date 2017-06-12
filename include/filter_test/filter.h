/*
 * filter.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_FILTER_H_
#define INCLUDE_FILTER_TEST_FILTER_H_

#include <list>
#include <iostream>

#include "filter_test/block.h"
#include "filter_test/defines.h"
#include "filter_test/measurement_manager.h"
#include "filter_test/residual.h"
#include "filter_test/state.h"
#include "filter_test/helper_functions.h"

struct ResidualContainer {
  std::vector<int> first_keys;
  std::vector<int> second_keys;
  std::vector<int> measurement_keys;
  ResidualBase* residual_;
};

class Filter {
 public:
  Filter():total_residual_dimension_(0) {}

  ~Filter() {
    for(ResidualContainer& current_residual:residuals_) {
      delete current_residual.residual_;
    }
  }

  // state related stuff
  std::vector<BlockType> state_types_;
  //std::vector<std::string> state_names_;

  State first_state_;
  State second_state_;

  MeasurementManager measurement_manager_;

  // everything related to the residuals
  std::vector<ResidualContainer> residuals_;
  int total_residual_dimension_;

  bool defineState(std::vector<BlockType> state_types);

  void initStateValue(const int key, const VectorXRef& value);

  // Adds a residual and takes ownership of the residual.
  // Classical KF measurements only depend on the second state and therefore only contain second_keys.
  bool addResidual(ResidualBase* residual, std::vector<int> first_keys,
                   std::vector<int> second_keys, std::vector<int> measurement_keys = std::vector<int>());

  void addMeasurement(int timeline_key, int timestamp_ns, MeasurementBase* measurement);


  void printState();

  void printTimeline();

  void printResiduals();

  void checkResiduals();

  void step();

 private:
  std::vector<BlockBase*> getBlocks(const State& state, const std::vector<int>& keys);
};



#endif /* INCLUDE_FILTER_TEST_FILTER_H_ */
