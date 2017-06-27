/*
 * filter.cpp
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#include <algorithm>  // Required for std::find

#include "filter_test/filter.h"
#include "filter_test/utils/logging.h"

namespace tsif {

void Filter::computeLinearizationPoint(const State& state, const int timestamp_ns) {
  temporary_second_state_ = state;
}

void Filter::constructProblem(const FilterProblemDescription& filter_problem, const State& first_state, const State& second_state, VectorX* residual_vector, MatrixX* jacobian_wrt_state1, MatrixX* jacobian_wrt_state2) {
  int index_residual = 0;
  for (ResidualContainer* residual_container : filter_problem.residual_containers_) {
    if (residual_container->residual->active_) {
      std::vector<BlockBase*> blocks1 = first_state.getBlocks(residual_container->first_keys);
      std::vector<BlockBase*> blocks2 = second_state.getBlocks(residual_container->second_keys);

      const int& residual_dimension = residual_container->residual->dimension_;
      VectorXRef residual_error = residual_vector->segment(index_residual, residual_dimension);
      std::vector<MatrixXRef> jacobian_wrt_state1_blocks =
          getJacobianBlocks(first_state, residual_container->first_keys, index_residual, residual_dimension, jacobian_wrt_state1);
      std::vector<MatrixXRef> jacobian_wrt_state2_blocks =
          getJacobianBlocks(first_state, residual_container->second_keys, index_residual, residual_dimension, jacobian_wrt_state2);

      bool residual_ok =
          residual_container->residual->evaluate(blocks1, blocks2, filter_problem.timestamp_previous_update_ns, filter_problem.timestamp_ns, &residual_error,
                                                 &jacobian_wrt_state1_blocks, &jacobian_wrt_state2_blocks);
      if (residual_ok) {
        index_residual += residual_dimension;
      }
    }
  }
}

// Most of this function is copied from Bloesch https://github.com/ethz-asl/two_state_information_filter!!!
void Filter::update(const FilterProblemDescription& filter_problem, const State& state, State* updated_state) {
  //  // Compute linearisation point
  const int& timestamp_ns = filter_problem.timestamp_ns;
  computeLinearizationPoint(state, timestamp_ns);
  //
  //  // Check available measurements and prepare residuals
  const int& active_residuals_dimension = filter_problem.residuals_dimension_;
  //  PreProcess();
  //
  // Temporaries
  residual_vector_.resize(active_residuals_dimension);
  residual_vector_.setZero();
  jacobian_wrt_state1_.resize(active_residuals_dimension, state.minimal_dimension_);
  jacobian_wrt_state1_.setZero();
  jacobian_wrt_state2_.resize(active_residuals_dimension, state.minimal_dimension_);
  jacobian_wrt_state2_.setZero();

  double weightedDelta = th_iter_;
  MatrixX newInf(state.minimal_dimension_, state.minimal_dimension_);
  size_t update_iteration;
  for (update_iteration = 0; update_iteration < max_iter_ && weightedDelta >= th_iter_; ++update_iteration) {
    constructProblem(filter_problem, state, temporary_second_state_, &residual_vector_, &jacobian_wrt_state1_, &jacobian_wrt_state2_);

    TSIF_LOG("Innovation:\t" << residual_vector_.transpose());
    TSIF_LOG("JacPre:\n" << jacobian_wrt_state1_);
    TSIF_LOG("JacCur:\n" << jacobian_wrt_state2_);

    // Compute Kalman Update // TODO use more efficient form
    MatrixX D = information_ + jacobian_wrt_state1_.transpose() * jacobian_wrt_state1_;
    MatrixX J(active_residuals_dimension, active_residuals_dimension);
    J.setIdentity();
#if TSIF_VERBOSE > 0
    Eigen::JacobiSVD<MatrixX> svdD(D);
    const double condD = svdD.singularValues()(0) / svdD.singularValues()(svdD.singularValues().size() - 1);
    TSIF_LOG("D condition number:\n" << condD);
#endif
    MatrixX S =
        jacobian_wrt_state2_.transpose() * (J - jacobian_wrt_state1_ * D.inverse() * jacobian_wrt_state1_.transpose());
    newInf = S * jacobian_wrt_state2_;
    newInf = 0.5 * (newInf + newInf.transpose().eval());
    Eigen::LDLT<MatrixX> I_LDLT(newInf);
#if TSIF_VERBOSE > 0
    Eigen::JacobiSVD<MatrixX> svdI(newInf);
    const double condI = svdI.singularValues()(0) / svdI.singularValues()(svdI.singularValues().size() - 1);
    TSIF_LOG("I condition number:\n" << condI);
#endif
    TSIF_LOGEIF((I_LDLT.info() != Eigen::Success), "Computation of Iinv failed");
    VectorX dx = -I_LDLT.solve(S * residual_vector_);

    // Apply Kalman Update
    temporary_second_state_.boxPlus(dx, &temporary_second_state_);

    weightedDelta = sqrt((dx.dot(newInf * dx)) / dx.size());
    TSIF_LOG("iter: " << update_iteration << "\tw: " << sqrt((dx.dot(dx)) / dx.size()) << "\twd: " << weightedDelta);
  }

  TSIF_LOGWIF(weightedDelta >= th_iter_, "Reached maximal iterations:" << update_iteration);

  *updated_state = temporary_second_state_;

  information_ = newInf;
  TSIF_LOG("State after Update:\n" << state.printState());
  TSIF_LOG("Information matrix:\n" << information_);

  //    // Post Processing
  //    PostProcess();
}

bool Filter::init(const State& state, const int& total_residual_dimension) {
  CHECK(total_residual_dimension > 0) << "residual dimension: " << std::to_string(total_residual_dimension);
  const int& minimal_state_dimension = state.minimal_dimension_;
  temporary_second_state_ = state;
  information_.resize(minimal_state_dimension, minimal_state_dimension);
  information_.setIdentity();
  residual_vector_.resize(total_residual_dimension);

  weightedDelta_.resize(minimal_state_dimension);
  jacobian_wrt_state1_.resize(total_residual_dimension, minimal_state_dimension);
  jacobian_wrt_state2_.resize(total_residual_dimension, minimal_state_dimension);
  return true;
}

}  // namespace tsif
