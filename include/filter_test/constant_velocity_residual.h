/*
 * constant_velocity_residual.h
 *
 *  Created on: 12.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_CONSTANT_VELOCITY_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_CONSTANT_VELOCITY_RESIDUAL_H_

namespace tsif {

// This residual implements the following set of equations
// r_x = ( x_k + dt * v_k) - x_kp1;
// r_v = ( v_k) - v_kp1;

// We assume that the residuals are embedded in a vector space (i.e. tangent space for manifolds).
class ConstantVelocityResidual : public ResidualBase {

  struct InternalState {
    InternalState(const std::vector<BlockBase*>& state):p(state[0]->getValue<VectorBlock<3>>()),v(state[1]->getValue<VectorBlock<3>>()) {}
    InternalState(Vector3& p1, Vector3& v1):p(p1),v(v1) {}
    Vector3& p;
    Vector3& v;
  };

  static const bool kIsMergeable = true;
  static const int kResidualDimension = 6;

 public:
  ConstantVelocityResidual(const double position_sigma, const double velocity_sigma)
      : ResidualBase(kResidualDimension, kIsMergeable) {
    sqrt_information_position_ = 1 / position_sigma;
    sqrt_information_velocity_ = 1 / velocity_sigma;
  }

  ~ConstantVelocityResidual() {}

  virtual bool prepareResidual(const int t1_ns, const int t2_ns) {
    // this residual has nothing to prepare.
    return true;
  }

  virtual bool predict(const std::vector<BlockBase*>& state, const int t1_ns,
                       const int t2_ns, std::vector<BlockBase*>* predicted_state, std::vector<MatrixXRef>* jacobian_wrt_state1) {
    CHECK_NOTNULL(predicted_state);
    const double dt = kNanoSecondsToSeconds * (t2_ns - t1_ns);

    const Vector3& p_k = state[0]->getValue<VectorBlock<3>>();
    const Vector3& v_k = state[1]->getValue<VectorBlock<3>>();
    Vector3& p_kp1 = (*predicted_state)[0]->getValue<VectorBlock<3>>();
    Vector3& v_kp1 = (*predicted_state)[1]->getValue<VectorBlock<3>>();

    p_kp1 = p_k + dt * v_k;
    v_kp1 = v_k;

    // For vector spaces this is identical to the one in evaluate().
    if (jacobian_wrt_state1 != NULL) {
      // d(r(...))/d(p_k)
      (*jacobian_wrt_state1)[0].template block<3, 3>(0, 0) = Matrix3::Identity();
      (*jacobian_wrt_state1)[0].template block<3, 3>(3, 0) = Matrix3::Zero();

      // d(r(...))/d(v_k)
      (*jacobian_wrt_state1)[1].template block<3, 3>(0, 0) = dt * Matrix3::Identity();
      (*jacobian_wrt_state1)[1].template block<3, 3>(3, 0) = Matrix3::Identity();
    }

    return true;
  }


  virtual bool evaluate(const std::vector<BlockBase*>& state1, const std::vector<BlockBase*>& state2, const int t1_ns,
                        const int t2_ns, VectorXRef* residual, std::vector<MatrixXRef>* jacobian_wrt_state1,
                        std::vector<MatrixXRef>* jacobian_wrt_state2) {
    const Vector3& p_k = state1[0]->getValue<VectorBlock<3>>();
    const Vector3& v_k = state1[1]->getValue<VectorBlock<3>>();
    const Vector3& p_kp1 = state2[0]->getValue<VectorBlock<3>>();
    const Vector3& v_kp1 = state2[1]->getValue<VectorBlock<3>>();

    if (residual == NULL) {
      return false;
    }

    double dt = kNanoSecondsToSeconds * (t2_ns - t1_ns);
    residual->template block<3, 1>(0, 0) = sqrt_information_position_ * ((p_k + dt * v_k) - p_kp1);
    residual->template block<3, 1>(3, 0) = sqrt_information_velocity_ * ((v_k) - v_kp1);

    if (jacobian_wrt_state1 != NULL) {
      // d(r(...))/d(p_k)
      (*jacobian_wrt_state1)[0].template block<3, 3>(0, 0) = sqrt_information_position_ * Matrix3::Identity();
      (*jacobian_wrt_state1)[0].template block<3, 3>(3, 0) = Matrix3::Zero();

      // d(r(...))/d(v_k)
      (*jacobian_wrt_state1)[1].template block<3, 3>(0, 0) = sqrt_information_position_ * dt * Matrix3::Identity();
      (*jacobian_wrt_state1)[1].template block<3, 3>(3, 0) = sqrt_information_velocity_ * Matrix3::Identity();
    }

    if (jacobian_wrt_state2 != NULL) {
      // d(r(...))/d(p_kp1)
      (*jacobian_wrt_state2)[0].template block<3, 3>(0, 0) = -sqrt_information_position_ * Matrix3::Identity();
      (*jacobian_wrt_state2)[0].template block<3, 3>(3, 0) = Matrix3::Zero();

      // d(r(...))/d(v_kp1)
      (*jacobian_wrt_state2)[1].template block<3, 3>(0, 0) = Matrix3::Zero();
      (*jacobian_wrt_state2)[1].template block<3, 3>(3, 0) = -sqrt_information_velocity_ * Matrix3::Identity();
    }

    return true;
  }

  virtual std::string getPrintableName() const { return "const velocity"; }

  virtual bool inputTypesValid(const std::vector<BlockBase*>& state1, const std::vector<BlockBase*>& state2) {
    bool all_types_ok = true;
    all_types_ok &= state1[0]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &= state1[1]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &= state2[0]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &= state2[1]->isBlockTypeCorrect<VectorBlock<3>>();
    TSIF_LOGEIF(!all_types_ok, "Constant velocity residual has wrong block types. Check your state indices!");
    return all_types_ok;
  }

 private:
  double sqrt_information_position_;
  double sqrt_information_velocity_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_CONSTANT_VELOCITY_RESIDUAL_H_ */
