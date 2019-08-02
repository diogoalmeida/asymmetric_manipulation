#include <asymmetric_manipulation/ext_rel_jac.hpp>

namespace coordination_algorithms
{
ExtRelJac::ExtRelJac(const Vector3d &pos_upper_ct, const Vector3d &pos_lower_ct,
                     double ori_ct, bool onlyl)
    : AlgorithmBase(pos_upper_ct, pos_lower_ct, ori_ct), onlyl_(onlyl)
{
}

Eigen::VectorXd ExtRelJac::control(const sensor_msgs::JointState &state,
                                   const Vector3d &r1, const Vector3d &r2,
                                   const Vector6d &abs_twist,
                                   const Vector6d &rel_twist)
{
  KDL::Frame p1, p2, eef1, eef2;
  Eigen::Affine3d p1_eig, p2_eig, eef1_eig, eef2_eig;

  kdl_manager_->getGrippingPoint(eef1_, state, p1);
  kdl_manager_->getGrippingPoint(eef2_, state, p2);
  kdl_manager_->getEefPose(eef1_, state, eef1);
  kdl_manager_->getEefPose(eef2_, state, eef2);

  tf::transformKDLToEigen(p1, p1_eig);
  tf::transformKDLToEigen(p2, p2_eig);
  tf::transformKDLToEigen(eef1, eef1_eig);
  tf::transformKDLToEigen(eef2, eef2_eig);

  Vector3d r1_conv = r1 + p1_eig.translation() - eef1_eig.translation(),
           r2_conv = r2 + p2_eig.translation() - eef2_eig.translation();

  Matrix12d W = computeW(r1_conv, r2_conv);
  KDL::Jacobian J1_kdl, J2_kdl;
  Eigen::MatrixXd J_sym, J_asym, J_comb, J, J_abs;

  kdl_manager_->getJacobian(eef1_, state, J1_kdl);
  kdl_manager_->getJacobian(eef2_, state, J2_kdl);

  J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
  J_abs = J;
  J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
  J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

  if (dynamic_alpha_)
  {
    alpha_ = computeAlpha(J1_kdl.data, J2_kdl.data);
  }

  J_asym = computeAsymJac(J, W, alpha_);
  J_abs = computeAsymAbsJac(J, W, alpha_);
  joint_manip_ = std::sqrt((J_asym * J_asym.transpose()).determinant());
  J_sym = computeAsymJac(J, W, 0.5);
  J_comb = Eigen::MatrixXd::Zero(J_asym.rows() + J_sym.rows(), J_asym.cols());
  J_comb.block(0, 0, J_sym.rows(), J_sym.cols()) = J_sym;
  J_comb.block(J_sym.rows(), 0, J_asym.rows(), J_asym.cols()) = J_asym;

  MatrixInvRelativeJacd damped_sim_inverse =
      J_sym.transpose() *
      (J_sym * J_sym.transpose() + damping_ * Matrix6d::Identity()).inverse();
  MatrixInvRelativeJacd damped_asym_inverse =
      J_asym.transpose() *
      (J_asym * J_asym.transpose() + damping_ * Matrix6d::Identity()).inverse();
  MatrixInvRelativeJacd damped_abs_inverse =
      J_abs.transpose() *
      (J_abs * J_abs.transpose() + damping_ * Matrix6d::Identity()).inverse();

  Eigen::VectorXd qdot_asym = damped_asym_inverse * rel_twist;
  Eigen::VectorXd qdot_sym = damped_sim_inverse * rel_twist;
  Eigen::VectorXd qdot_abs = damped_abs_inverse * abs_twist;
  Eigen::VectorXd qdot;

  if (!onlyl_)
  {
    qdot = qdot_sym +
           (Matrix14d::Identity() - damped_sim_inverse * J_sym) * qdot_asym;

    if (use_sec_abs_)
    {
      qdot += (Matrix14d::Identity() - damped_sim_inverse * J_sym) * qdot_abs;
    }
  }
  else
  {
    qdot = qdot_asym +
           (Matrix14d::Identity() - damped_asym_inverse * J_asym) * qdot_abs;
  }

  Vector12d joint_twist = W * J * qdot;
  v1_ = joint_twist.block<6, 1>(0, 0);
  v2_ = joint_twist.block<6, 1>(6, 0);

  return qdot;
}

Eigen::MatrixXd ExtRelJac::computeAsymJac(const Eigen::MatrixXd &J,
                                          const Matrix12d &W,
                                          double alpha) const
{
  MatrixRelLinkingd L = MatrixRelLinkingd::Zero();
  double scaling = 1 / ((1 - alpha) * (1 - alpha) + alpha * alpha);

  L.block<6, 6>(0, 0) = -(1 - alpha) * scaling * Matrix6d::Identity();
  L.block<6, 6>(0, 6) = alpha * scaling * Matrix6d::Identity();

  return L * W * J;
}

Eigen::MatrixXd ExtRelJac::computeAsymAbsJac(const Eigen::MatrixXd &J,
                                             const Matrix12d &W,
                                             double alpha) const
{
  MatrixRelLinkingd L = MatrixRelLinkingd::Zero();

  L.block<6, 6>(0, 0) = alpha * Matrix6d::Identity();
  L.block<6, 6>(0, 6) = (1 - alpha) * Matrix6d::Identity();

  return L * W * J;
}

double ExtRelJac::computeAlpha(const Eigen::MatrixXd &J1,
                               const Eigen::MatrixXd &J2) const
{
  double mu1 = std::sqrt((J1 * J1.transpose()).determinant());
  double mu2 = std::sqrt((J2 * J2.transpose()).determinant());

  return mu2 / (mu1 + mu2);
}

}  // namespace coordination_algorithms
