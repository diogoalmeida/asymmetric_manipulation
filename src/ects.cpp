#include <coordination_algorithms/ects.hpp>

namespace coordination_algorithms
{
ECTS::ECTS() : AlgorithmBase() {}

Eigen::VectorXd ECTS::control(const sensor_msgs::JointState &state,
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
  Matrix12d L = Matrix12d::Zero();
  Vector12d total_twist;
  Eigen::VectorXd q_dot;
  Eigen::MatrixXd J_e, J;
  KDL::Jacobian J1_kdl, J2_kdl;

  kdl_manager_->getJacobian(eef1_, state, J1_kdl);
  kdl_manager_->getJacobian(eef2_, state, J2_kdl);

  L.block<6, 6>(0, 0) = abs_alpha_ * Matrix6d::Identity();
  L.block<6, 6>(0, 6) = (1 - abs_alpha_) * Matrix6d::Identity();
  L.block<6, 6>(6, 0) = -Matrix6d::Identity();
  L.block<6, 6>(6, 6) = Matrix6d::Identity();

  J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
  J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
  J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

  J_e = L * W * J;

  Matrix12d damped_inverse =
      (J_e * J_e.transpose() + damping_ * Matrix12d::Identity());

  total_twist.block<6, 1>(0, 0) = abs_twist;
  total_twist.block<6, 1>(6, 0) = rel_twist;
  q_dot =
      J_e.transpose() * damped_inverse.colPivHouseholderQr().solve(total_twist);

  return q_dot;
}
}  // namespace coordination_algorithms
