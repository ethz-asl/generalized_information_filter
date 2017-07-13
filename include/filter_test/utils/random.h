#ifndef INCLUDE_FILTER_TEST_UTILS_RANDOM_H_
#define INCLUDE_FILTER_TEST_UTILS_RANDOM_H_

// This file is adapted from https://github.com/ethz-asl/two_state_information_filter

#include <random>

#include "filter_test/defines.h"

namespace tsif {

/*! \brief Normal Random Number Generator
 *         Singleton class for generating normal random numbers (N(0,1)). Allows setting of seed.
 */
class NormalRandomNumberGenerator{
 public:

  NormalRandomNumberGenerator(NormalRandomNumberGenerator const&) = delete;
  void operator=(NormalRandomNumberGenerator const&) = delete;

  void setSeed(int s){
    generator_.seed(s);
  }
  double getRandomNumber(){
    return distribution_(generator_);
  }
  template<int N>
  Vector<N> getVector(){
    Vector<N> n;
    for(int i=0;i<N;i++){
      n(i) = getRandomNumber();
    }
    return n;
  }
  static NormalRandomNumberGenerator& getInstance(){
    static NormalRandomNumberGenerator instance;
    return instance;
  }
  std::default_random_engine& getGenerator(){
    return generator_;
  }
 protected:
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
  NormalRandomNumberGenerator(): generator_(0), distribution_(0.0,1.0){
  }
};

} // namespace tsif

#endif /* INCLUDE_FILTER_TEST_UTILS_RANDOM_H_ */
