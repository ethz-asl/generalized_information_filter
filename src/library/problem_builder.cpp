/*
 * problem_builder.cpp
 *
 *  Created on: 26.06.2017
 *      Author: burrimi
 */

#include "filter_test/problem_builder.h"
#include "filter_test/helper_functions.h"

namespace tsif {



bool ProblemBuilder::addResidual(ResidualBase* residual, std::vector<int> first_keys, std::vector<int> second_keys,
                         std::vector<int> measurement_keys) {

  ResidualContainer container;
  container.first_keys = first_keys;
  container.second_keys = second_keys;
  container.residual = residual;

  total_residual_dimension_ += residual->dimension_;
  residual_containers_.push_back(container);
  return true;
}

void ProblemBuilder::printResiduals(const State& state) const {
  for (size_t i = 0; i < state.numberOfBlocks(); ++i) {
    std::string state_name = "S" + std::to_string(i);
    std::cout << padTo(state_name, 5);
  }
  std::cout << padTo("residual", 20);
  for (size_t i = 0; i < state.numberOfBlocks(); ++i) {
    std::string state_name = "S" + std::to_string(i);
    std::cout << padTo(state_name, 5);
  }
  std::cout << std::endl;

  for (const ResidualContainer& current_residual : residual_containers_) {
    for (size_t i = 0; i < state.numberOfBlocks(); ++i) {
      if (vectorContainsValue(current_residual.first_keys, i)) {
        std::cout << "  X  ";
      } else {
        std::cout << "     ";
      }
    }
    std::string residual_name = current_residual.residual->getPrintableName();
    std::cout << padTo(residual_name, 20);
    for (size_t i = 0; i < state.numberOfBlocks(); ++i) {
      if (vectorContainsValue(current_residual.second_keys, i)) {
        std::cout << "  X  ";
      } else {
        std::cout << "     ";
      }
    }
    std::cout << std::endl;
  }
}

void ProblemBuilder::checkResiduals(const State& state) const {
  for (const ResidualContainer& current_residual : residual_containers_) {
    std::vector<BlockBase*> blocks1 = state.getBlocks(current_residual.first_keys);
    std::vector<BlockBase*> blocks2 = state.getBlocks(current_residual.second_keys);
    std::cout << "Checking input types for residual : " << current_residual.residual->getPrintableName() << std::endl;
    current_residual.residual->inputTypesValid(blocks1, blocks2);
  }
}

// TODO(burrimi): This function currently ignores the available measurements and just iterates through all the residuals to check wether they are active.
FilterProblemDescription ProblemBuilder::getFilterProblemDescription(const UpdateDescription& update_description) {
  FilterProblemDescription filter_problem_description;
  filter_problem_description.timestamp_ns=update_description.timestamp_ns;
  filter_problem_description.timestamp_previous_update_ns=update_description.timestamp_previous_update_ns;

  int active_residuals_dimension = 0;
  for (ResidualContainer& current_residual : residual_containers_) {
    bool residual_ok = current_residual.residual->prepareResidual(update_description.timestamp_previous_update_ns, update_description.timestamp_ns);
    current_residual.residual->active_ = residual_ok;  // TODO(burrimi): Do this a better way. move to base class?

    if (residual_ok) {
      active_residuals_dimension += current_residual.residual->dimension_;
      filter_problem_description.residual_containers_.emplace_back(&current_residual);
    }
  }
  filter_problem_description.residuals_dimension_ = active_residuals_dimension;
  return filter_problem_description;
}


}  // namespace tsif
