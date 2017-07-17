/*
 * test_geometry.cpp
 *
 *  Created on: 17.07.2017
 *      Author: burrimi
 */

#include "eigen-checks/gtest.h"
#include "gtest/gtest.h"

#include "filter_test/utils/geometry.h"

namespace tsif {

TEST(Geometry, TestQuaternion) {
  // Initial state values.
  Quaternion q(Quaternion::Identity().slerp(
      0.10, Quaternion::FromTwoVectors(Vector3(1, 0, 0), Vector3(0, 0, 1))));

  Quaternion p(Quaternion::Identity().slerp(
      0.20, Quaternion::FromTwoVectors(Vector3(1, 0, 0), Vector3(0, 0, 1))));

  Vector3 theata(0.2, 0.3, -0.1);

  // Check: (q boxplus theata) boxminus q = theata
  Quaternion q_rot;
  Vector3 theta_new;
  quaternion_helpers::boxPlus(q, theata, &q_rot);
  quaternion_helpers::boxMinus(q_rot, q, &theta_new);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(theata, theta_new));

  // Check: (q boxplus theata) boxplus -theata = q
  Quaternion q_orig;
  Vector3 theta_neg = -theata;
  quaternion_helpers::boxPlus(q_rot, theta_neg, &q_orig);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(q.vec(), q_orig.vec()));

  // Check: q boxplus (p boxminus q) = p
  Vector3 theta_temp;
  Quaternion p_orig;
  quaternion_helpers::boxMinus(p, q, &theta_temp);
  quaternion_helpers::boxPlus(q, theta_temp, &p_orig);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(p.vec(), p_orig.vec()));
}

}  // namespace tsif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
