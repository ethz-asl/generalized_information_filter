/*
 * defines.h
 *
 *  Created on: 01.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_DEFINES_H_
#define INCLUDE_FILTER_TEST_DEFINES_H_

#define DO_SANITY_CHECKS true

#include <Eigen/Dense>


template <int Dimension>
using Vector = Eigen::Matrix<double, Dimension, 1>;

typedef Eigen::Vector3d Vector3;

typedef Eigen::Ref<Eigen::Vector3d> Vector3Ref;
typedef Eigen::Ref<Eigen::VectorXd> VectorXRef;

typedef Eigen::Ref<Eigen::MatrixXd> MatrixXRef;




constexpr int kMaxMeasurmenetBufferSize = 1000;

#endif /* INCLUDE_FILTER_TEST_DEFINES_H_ */
