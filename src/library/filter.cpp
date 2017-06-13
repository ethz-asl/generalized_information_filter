/*
 * filter.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include <algorithm> // Required for std::find

#include "filter_test/filter.h"


//bool isOrdered(const std::vector<int>& vector)
//{
//  std::vector<int>::iterator it = vector.begin();
//  int smallest_number = *it;
//  while(it!=vector.end()) {
//    ++it;
//    if(*it <= smallest_number) {
//      return false;
//    }
//    smallest_number = *it;
//  }
//  return true;
//}

bool Filter::defineState(std::vector<BlockType> state_types) {
  state_types_ = state_types;
  first_state_.defineState(state_types);
  second_state_.defineState(state_types);
  return true;
}


bool Filter::addResidual(ResidualBase* residual, std::vector<int> first_keys,
                         std::vector<int> second_keys, std::vector<int> measurement_keys) {
  CHECK(first_state_.dimension_ > 0); // Check if state is defined.
  ResidualContainer container;
  container.first_keys = first_keys;
  container.second_keys = second_keys;
  container.residual_ = residual;

  std::vector<Timeline*> timelines = measurement_manager_.getTimelines(measurement_keys, residual->is_mergeable_);
  residual->setMeasurementTimelines(timelines);

  total_residual_dimension_ += residual->dimension_;
  residuals_.push_back(container);
  return true;
}

void Filter::addMeasurement(int timeline_key, int timestamp_ns, MeasurementBase* measurement) {
  measurement_manager_.addMeasurement(timeline_key, timestamp_ns, measurement);

#ifdef VERBOSE_MODE
  std::cout << "added " << measurement->getPrintableMeasurement() << " at time "<< std::to_string(timestamp_ns) << std::endl;
#endif
  int timestamp_update_ns;
  if(!measurement_manager_.shouldIRunTheFilter(timestamp_previous_update_ns_, &timestamp_update_ns)) {
    return;
  }

  if(!first_run_) {
#ifdef VERBOSE_MODE
    std::cout << "update at time "<< std::to_string(timestamp_update_ns) << std::endl;
#endif
    update(timestamp_update_ns);
    return;
  }

  // This is the first run and we have to initialize everything
#ifdef VERBOSE_MODE
  std::cout << "first run at time "<< std::to_string(timestamp_update_ns) << std::endl;
#endif
  init();
  update(timestamp_update_ns);
  first_run_ = false;

}

void Filter::update(const int timestamp_ns) {
  int index_residual = 0;
  jacobian_wrt_state1_.setZero();
  jacobian_wrt_state2_.setZero();
  residual_vector_.setZero();
  if(timestamp_ns > -1) {
    for(ResidualContainer& current_residual:residuals_) {
      std::vector<BlockBase*> blocks1 = getBlocks(first_state_, current_residual.first_keys);
      std::vector<BlockBase*> blocks2 = getBlocks(second_state_, current_residual.second_keys);
      
      const int& residual_dimension = current_residual.residual_->dimension_;
      VectorXRef residual = residual_vector_.segment(index_residual, residual_dimension);
      std::vector<MatrixXRef> jacobian_wrt_state1 = getJacobianBlocks(jacobian_wrt_state1_, current_residual.first_keys, index_residual, residual_dimension);
      std::vector<MatrixXRef> jacobian_wrt_state2 = getJacobianBlocks(jacobian_wrt_state2_, current_residual.second_keys, index_residual, residual_dimension);;
      
      bool residual_ok = current_residual.residual_->evaluate(blocks1, blocks2, timestamp_previous_update_ns_, timestamp_ns, &residual, &jacobian_wrt_state1, &jacobian_wrt_state2);
      if(residual_ok) {
        std::cout << "r idx " << index_residual << std::endl;

        index_residual += residual_dimension;
      }
    }
  }

  std::cout << "J1 " << jacobian_wrt_state1_ << std::endl;
  std::cout << "J2 " << jacobian_wrt_state2_ << std::endl;
  std::cout << "r  " << residual_vector_.transpose() << std::endl;
  
  timestamp_previous_update_ns_ = timestamp_ns;
}

void Filter::initStateValue(const int key, const VectorXRef& value) {
  first_state_.setState(key, value);
}

void Filter::printState() {
  first_state_.printState();
}

void Filter::printTimeline() {
  measurement_manager_.printTimeline();
}

void Filter::printResiduals() {
  for(size_t i=0; i < first_state_.state_blocks_.size(); ++i) {
    std::string state_name = "S" + std::to_string(i);
    std::cout << padTo(state_name, 5);
  }
  std::cout << padTo("residual", 20);
  for(size_t i=0; i < first_state_.state_blocks_.size(); ++i) {
    std::string state_name = "S" + std::to_string(i);
    std::cout << padTo(state_name, 5);
  }
  std::cout << std::endl;

  for(ResidualContainer& current_residual:residuals_) {
    for(size_t i=0; i < first_state_.state_blocks_.size(); ++i) {
      if (vectorContainsValue(current_residual.first_keys, i)) {
        std::cout << "  X  ";
      } else {
        std::cout << "     ";
      }
    }
    std::string residual_name = current_residual.residual_->getResidualName();
    std::cout << padTo(residual_name, 20);
    for(size_t i=0; i < second_state_.state_blocks_.size(); ++i) {
      if (vectorContainsValue(current_residual.second_keys, i)) {
        std::cout << "  X  ";
      } else {
        std::cout << "     ";
      }
    }
    std::cout << std::endl;
  }
}

void Filter::checkResiduals() {
  for(ResidualContainer& current_residual:residuals_) {
    std::vector<BlockBase*> blocks1 = getBlocks(first_state_, current_residual.first_keys);
    std::vector<BlockBase*> blocks2 = getBlocks(second_state_, current_residual.second_keys);
    std::cout << "Checking input types for residual : " <<
        current_residual.residual_->getResidualName() << std::endl;
    current_residual.residual_->inputTypesValid(blocks1,blocks2);
  }
}



bool Filter::init() {
  const int& minimal_state_dimension = first_state_.minimal_dimension_;
  information_.resize(minimal_state_dimension, minimal_state_dimension);
  information_.setIdentity();
  residual_vector_.resize(total_residual_dimension_);
  jacobian_wrt_state1_.resize(total_residual_dimension_, minimal_state_dimension);
  jacobian_wrt_state2_.resize(total_residual_dimension_, minimal_state_dimension);
  return true;
}
