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

  bool finiteDifference() {
    const int outDim = Out::Dim();
    const int I = std::tuple_element<N,std::tuple<Ins...>>::type::template GetId<C>();
    const int inDim = std::tuple_element<N,std::tuple<Ins...>>::type::template GetElementDim<I>();
    Out outRef,outDis;
    Eval(outRef,insRef);
    std::tuple<Ins...> insDis = insRef;
    Vec<inDim> inDif;
    Vec<outDim> outDif;
    for(unsigned int j=0;j<inDim;j++){
      inDif.setZero();
      inDif(j) = d;
      std::get<N>(insRef).template GetElement<I>().Boxplus(inDif,std::get<N>(insDis).template GetElement<I>());
      Eval(outDis,insDis);
      outDis.Boxminus(outRef,outDif);
      J.col(std::get<N>(insRef).Start(I)+j) = outDif/d;
    }
  }

  int JacTest(double th, double d, const std::tuple<typename Ins::CRef...> ins){
     const int outDim = Out::Dim();
     const int inDim = std::get<N>(ins).Dim();
     if(outDim > 0 & inDim > 0){
       MatX J(outDim,inDim);
       J.setZero();
       MatX J_FD(outDim,inDim);
       J_FD.setZero();
       Jac<N>(J,ins);
       JacFindifFull<N>(d,J_FD,ins);
       TSIF_LOG("Analytical Jacobian:\n" << J);
       TSIF_LOG("Numerical Jacobian:\n" << J_FD);
       typename MatX::Index maxRow, maxCol = 0;
       const double r = (J-J_FD).array().abs().maxCoeff(&maxRow, &maxCol);
       if(r>th){
         std::cout << "\033[31m==== Model jacInput (" << N << ") Test failed: " << r
                   << " is larger than " << th << " at row "
                   << maxRow << " and col " << maxCol << " ===="
                   << "  " << J(maxRow,maxCol) << " vs " << J_FD(maxRow,maxCol)
                   << "\033[0m" << std::endl;
         return 1;
       } else{
         std::cout << "\033[32m==== Test successful (" << r << ") ====\033[0m" << std::endl;
         return 0;
       }
     } else {
       std::cout << "\033[32m==== Test successful ( dimension 0 ) ====\033[0m" << std::endl;
     }
   }

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUAL_H_ */
