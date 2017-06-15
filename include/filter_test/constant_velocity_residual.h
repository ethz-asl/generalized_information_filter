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
// r_x = x_kp1 - ( x_k + dt * v_k);
// r_v = v_kp1 - ( v_k);

// We assume that the residuals are embedded in a vector space (i.e. tangent space for manifolds).
class ConstantVelocityResidual: public ResidualBase {
  static const bool kIsMergeable = true;
  static const int kResidualDimension = 6;
public:
  ConstantVelocityResidual(const double position_sigma, const double velocity_sigma): ResidualBase(kResidualDimension, kIsMergeable) {
    state1_block_types_ = {BlockType::kVector3, BlockType::kVector3};
    state2_block_types_ = {BlockType::kVector3, BlockType::kVector3};

    sqrt_information_position_ = 1 / position_sigma;
    sqrt_information_velocity_ = 1 / velocity_sigma;
  }

  ~ConstantVelocityResidual() {}

  virtual bool prepareResidual(const int t1_ns, const int t2_ns) {
    // this residual has nothing to prepare.
    return true;
  }

  virtual bool evaluate(const std::vector<BlockBase*>& state1,
                        const std::vector<BlockBase*>& state2,
                        const int t1_ns, const int t2_ns,
                        VectorXRef* residual, std::vector<MatrixXRef>* jacobian_wrt_state1,
                        std::vector<MatrixXRef>* jacobian_wrt_state2) {

    const Vector3& p_k = static_cast<VectorBlock<3>*>(state1[0])->value_;
    const Vector3& v_k = static_cast<VectorBlock<3>*>(state1[1])->value_;
    const Vector3& p_kp1 = static_cast<VectorBlock<3>*>(state2[0])->value_;
    const Vector3& v_kp1 = static_cast<VectorBlock<3>*>(state2[1])->value_;

    if(residual == NULL) {
      return false;
    }

    double dt = kNanoSecondsToSeconds * (t2_ns - t1_ns);
    residual->template block<3,1>(0,0) = sqrt_information_position_ * (p_kp1 - (p_k - dt * v_k));
    residual->template block<3,1>(3,0) = sqrt_information_velocity_ * (v_kp1 - (v_k));

    if(jacobian_wrt_state1 != NULL) {
      (*jacobian_wrt_state1)[0].template block<3,3>(0,0) = -sqrt_information_position_ * Matrix3::Identity();
      (*jacobian_wrt_state1)[0].template block<3,3>(3,0) = Matrix3::Zero();

      (*jacobian_wrt_state1)[1].template block<3,3>(0,0) = -sqrt_information_position_ * dt * Matrix3::Identity();
      (*jacobian_wrt_state1)[1].template block<3,3>(3,0) = -sqrt_information_velocity_ * Matrix3::Identity();
    }

    if(jacobian_wrt_state2 != NULL) {
      (*jacobian_wrt_state2)[0].template block<3,3>(0,0) = sqrt_information_position_ * Matrix3::Identity();
      (*jacobian_wrt_state2)[0].template block<3,3>(3,0) = Matrix3::Zero();

      (*jacobian_wrt_state2)[1].template block<3,3>(0,0) = Matrix3::Zero();
      (*jacobian_wrt_state2)[1].template block<3,3>(3,0) = sqrt_information_velocity_ * Matrix3::Identity();
    }

    return true;
  }

  virtual std::string getResidualName() const {return "const velocity";}

private:

  double sqrt_information_position_;
  double sqrt_information_velocity_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_CONSTANT_VELOCITY_RESIDUAL_H_ */
