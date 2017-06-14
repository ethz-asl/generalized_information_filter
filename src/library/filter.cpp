/*
 * filter.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include <algorithm> // Required for std::find

#include "filter_test/filter.h"
#include "filter_test/utils/logging.h"


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

void Filter::computeLinearizationPoint(const int timestamp_ns) {
  second_state_ = first_state_;
}

int Filter::preProcessResidual(const int timestamp_ns) {
  int active_residuals_dimension = 0;
  for(ResidualContainer& current_residual:residuals_) {
    bool residual_ok = current_residual.residual_->prepareResidual(timestamp_previous_update_ns_, timestamp_ns);
    current_residual.residual_->active_ = residual_ok; // TODO(burrimi): Do this a better way. move to base class?

    if(residual_ok) {
      active_residuals_dimension += current_residual.residual_->dimension_;
    }
  }

  return active_residuals_dimension;
}

void Filter::constructProblem(const int timestamp_ns) {
  int index_residual = 0;
  for(ResidualContainer& current_residual:residuals_) {
    if(current_residual.residual_->active_) {
      std::vector<BlockBase*> blocks1 = getBlocks(first_state_, current_residual.first_keys);
      std::vector<BlockBase*> blocks2 = getBlocks(second_state_, current_residual.second_keys);

      const int& residual_dimension = current_residual.residual_->dimension_;
      VectorXRef residual = residual_vector_.segment(index_residual, residual_dimension);
      std::vector<MatrixXRef> jacobian_wrt_state1 = getJacobianBlocks(jacobian_wrt_state1_, current_residual.first_keys, index_residual, residual_dimension);
      std::vector<MatrixXRef> jacobian_wrt_state2 = getJacobianBlocks(jacobian_wrt_state2_, current_residual.second_keys, index_residual, residual_dimension);;

      bool residual_ok = current_residual.residual_->evaluate(blocks1, blocks2, timestamp_previous_update_ns_, timestamp_ns, &residual, &jacobian_wrt_state1, &jacobian_wrt_state2);
      if(residual_ok) {
        index_residual += residual_dimension;
      }
    }
  }
}

void Filter::update(const int timestamp_ns) {
//  jacobian_wrt_state1_.setZero();
//  jacobian_wrt_state2_.setZero();
//  residual_vector_.setZero();
//
//
//  std::cout << "J1 " << jacobian_wrt_state1_ << std::endl;
//  std::cout << "J2 " << jacobian_wrt_state2_ << std::endl;
//  std::cout << "r  " << residual_vector_.transpose() << std::endl;

  //
  //  // Compute linearisation point
  computeLinearizationPoint(timestamp_ns);
  //
  //  // Check available measurements and prepare residuals

  int innDim = preProcessResidual(timestamp_ns);
  //  PreProcess();
  //
    // Temporaries
    residual_vector_.resize(innDim);
    residual_vector_.setZero();
    jacobian_wrt_state1_.resize(innDim, first_state_.minimal_dimension_);
    jacobian_wrt_state1_.setZero();
    jacobian_wrt_state2_.resize(innDim, first_state_.minimal_dimension_);
    jacobian_wrt_state2_.setZero();

    double weightedDelta = th_iter_;
    MatrixX newInf(first_state_.minimal_dimension_,first_state_.minimal_dimension_);
    for(iter_=0;iter_<max_iter_ && weightedDelta >= th_iter_;++iter_){

      constructProblem(timestamp_ns);

      TSIF_LOG("Innovation:\t" << residual_vector_.transpose());
      TSIF_LOG("JacPre:\n" << jacobian_wrt_state1_);
      TSIF_LOG("JacCur:\n" << jacobian_wrt_state2_);

      // Compute Kalman Update // TODO use more efficient form
      MatrixX D = information_ + jacobian_wrt_state1_.transpose() * jacobian_wrt_state1_;
      MatrixX J(innDim,innDim); J.setIdentity();
  #if TSIF_VERBOSE > 0
      Eigen::JacobiSVD<MatrixX> svdD(D);
      const double condD = svdD.singularValues()(0) / svdD.singularValues()(svdD.singularValues().size()-1);
      TSIF_LOG("D condition number:\n" << condD);
  #endif
      MatrixX S = jacobian_wrt_state2_.transpose() * (J - jacobian_wrt_state1_ * D.inverse() * jacobian_wrt_state1_.transpose());
      newInf = S * jacobian_wrt_state2_;
      newInf = 0.5*(newInf + newInf.transpose().eval());
      Eigen::LDLT<MatrixX> I_LDLT(newInf);
  #if TSIF_VERBOSE > 0
      Eigen::JacobiSVD<MatrixX> svdI(newInf);
      const double condI = svdI.singularValues()(0) / svdI.singularValues()(svdI.singularValues().size()-1);
      TSIF_LOG("I condition number:\n" << condI);
  #endif
      TSIF_LOGEIF((I_LDLT.info() != Eigen::Success),"Computation of Iinv failed");
      VectorX dx = -I_LDLT.solve(S * residual_vector_);


      // Apply Kalman Update
      State newState = second_state_;

//      State newState = second_state_; TODO(burrimi): delete default copy constructor!!!

      second_state_.boxPlus(dx, &newState);

      second_state_ = newState;

      weightedDelta = sqrt((dx.dot(newInf*dx))/dx.size());
      TSIF_LOG("iter: " << iter_ << "\tw: " << sqrt((dx.dot(dx))/dx.size()) << "\twd: " << weightedDelta);
    }


    TSIF_LOGWIF(weightedDelta >= th_iter_, "Reached maximal iterations:" << iter_);

    first_state_ = second_state_;

    information_ = newInf;
    TSIF_LOG("State after Update:\n" << first_state_.printState());
    TSIF_LOG("Information matrix:\n" << information_);

//    // Post Processing
//    PostProcess();
//    time_ = t;


  timestamp_previous_update_ns_ = timestamp_ns;
}

void Filter::initStateValue(const int key, const VectorXRef& value) {
  first_state_.setState(key, value);
}

void Filter::printState() {
  std::cout << first_state_.printState() << std::endl;
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

  weightedDelta_.resize(minimal_state_dimension);
  jacobian_wrt_state1_.resize(total_residual_dimension_, minimal_state_dimension);
  jacobian_wrt_state2_.resize(total_residual_dimension_, minimal_state_dimension);
  return true;
}
