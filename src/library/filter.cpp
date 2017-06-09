/*
 * filter.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include <algorithm> // Required for std::find

#include "filter_test/filter.h"

std::string padTo(const std::string &str, const size_t num, const char paddingChar = ' ')
{
  std::string padded_string = str;
    if(num > str.size())
      padded_string.insert(padded_string.size(), num - padded_string.size(), paddingChar);
    return padded_string;
}

bool vectorContainsValue(const std::vector<int>& vec, int value) {
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}

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


bool Filter::addResidual(ResidualBase* residual, std::vector<int> first_keys,  std::vector<int> second_keys) {
  CHECK(first_state_.dimension_ > 0); // Check if state is defined.
  ResidualContainer container;
  container.first_keys = first_keys;
  container.second_keys = second_keys;
  container.residual_ = residual;

  total_residual_dimension_ += residual->dimension_;
  residuals_.push_back(container);
  return true;
}

void Filter::initStateValue(const int key, const VectorXRef& value) {
  first_state_.setState(key, value);
}

void Filter::printState() {
  first_state_.printState();
}

bool Filter::checkResiduals() {
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

  for(ResidualContainer& current_residual:residuals_) {
    std::vector<BlockBase*> blocks1 = getBlocks(first_state_, current_residual.first_keys);
    std::vector<BlockBase*> blocks2 = getBlocks(second_state_, current_residual.second_keys);
    std::cout << "Checking input types for residual : " <<
        current_residual.residual_->getResidualName() << std::endl;
    current_residual.residual_->inputTypesValid(blocks1,blocks2);
  }
}

std::vector<BlockBase*> Filter::getBlocks(const State& state, const std::vector<int>& keys) {
  std::vector<BlockBase*> blocks;
  for(int current_key:keys) {
    blocks.emplace_back(state.state_blocks_[current_key]);
  }
  return blocks;
}

