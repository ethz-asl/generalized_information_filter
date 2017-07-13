/*
 * defines.h
 *
 *  Created on: 01.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_DEFINES_H_
#define INCLUDE_FILTER_TEST_DEFINES_H_

#include <Eigen/Dense>

namespace tsif {

#define TSIF_VERBOSE true // Should we output a lot of potentially useless stuff to the console?

template <int Dimension>
using Vector = Eigen::Matrix<double, Dimension, 1>;

typedef Eigen::Vector3d Vector3;
typedef Eigen::VectorXd VectorX;

typedef Eigen::Ref<Eigen::Vector3d> Vector3Ref;
typedef Eigen::Ref<Eigen::VectorXd> VectorXRef;

typedef Eigen::Ref<Eigen::MatrixXd> MatrixXRef;

typedef Eigen::Matrix3d Matrix3;
typedef Eigen::MatrixXd MatrixX;

constexpr double kNanoSecondsToSeconds = 1e-9;
constexpr int kMaxMeasurementBufferSize = 1000;

constexpr int kRandomNumberSeed = 210;

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_DEFINES_H_ */
