#include <coordination_algorithms/ects.hpp>

namespace coordination_algorithms
{
ECTS::ECTS() : AlgorithmBase() {}

Eigen::VectorXd ECTS::control(const sensor_msgs::JointState &state,
                              const Vector3d &r1, const Vector3d &r2,
                              const Vector6d &abs_twist,
                              const Vector6d &rel_twist)
{
  Eigen::MatrixXd J;
  Matrix12d damped_inverse;
  Vector12d total_twist;
  Eigen::VectorXd q_dot;
  KDL::Frame p1, p2, eef1, eef2;
  Eigen::Affine3d p1_eig, p2_eig, eef1_eig, eef2_eig;

  total_twist.block<6, 1>(0, 0) = abs_twist;
  total_twist.block<6, 1>(6, 0) = rel_twist;

  kdl_manager_->getGrippingPoint(eef1_, state, p1);
  kdl_manager_->getGrippingPoint(eef2_, state, p2);
  kdl_manager_->getEefPose(eef1_, state, eef1);
  kdl_manager_->getEefPose(eef2_, state, eef2);

  tf::transformKDLToEigen(p1, p1_eig);
  tf::transformKDLToEigen(p2, p2_eig);
  tf::transformKDLToEigen(eef1, eef1_eig);
  tf::transformKDLToEigen(eef2, eef2_eig);

  J = computeJacobian(state, r1 + p1_eig.translation() - eef1_eig.translation(),
                      r2 + p2_eig.translation() - eef2_eig.translation());

  damped_inverse = (J * J.transpose() + damping_ * Matrix12d::Identity());

  q_dot =
      J.transpose() * damped_inverse.colPivHouseholderQr().solve(total_twist);

  return q_dot;
}

void ECTS::getAbsoluteVelocity(const sensor_msgs::JointState &state,
                               const Vector3d &r1, const Vector3d &r2,
                               Vector6d &abs_vel) const
{
  KDL::Twist v1_eef, v2_eef;
  Vector6d v1_eef_eig, v2_eef_eig, v1, v2;
  Vector12d total_twist;
  Eigen::Matrix<double, 6, 12> absL = Eigen::Matrix<double, 6, 12>::Zero();
  Matrix12d W = computeW(r1, r2);
  kdl_manager_->getGrippingTwist(eef1_, state, v1_eef);
  kdl_manager_->getGrippingTwist(eef2_, state, v2_eef);

  tf::twistKDLToEigen(v1_eef, v1_eef_eig);
  tf::twistKDLToEigen(v2_eef, v2_eef_eig);

  total_twist.block<6, 1>(0, 0) = v1_eef_eig;
  total_twist.block<6, 1>(6, 0) = v2_eef_eig;

  absL.block<6, 6>(0, 0) = abs_alpha_ * Matrix6d::Identity();
  absL.block<6, 6>(0, 6) = (1 - abs_alpha_) * Matrix6d::Identity();

  abs_vel = absL * W * total_twist;
}

KDL::Frame ECTS::getAbsoluteMotionFrame(const KDL::Frame &obj1,
                                        const KDL::Frame &obj2) const
{
  Eigen::Quaterniond obj1_orientation, relative_orientation;

  tf::quaternionKDLToEigen(obj1.M.Inverse() * obj2.M, relative_orientation);
  tf::quaternionKDLToEigen(obj1.M, obj1_orientation);

  Eigen::AngleAxisd relative_angle_axis(relative_orientation);
  Eigen::AngleAxisd absolute_angle_axis(
      (1 - abs_alpha_) * relative_angle_axis.angle(),
      relative_angle_axis.axis());

  Eigen::Quaterniond absolute_orientation(obj1_orientation *
                                          absolute_angle_axis);

  KDL::Frame absolute_frame;

  tf::quaternionEigenToKDL(absolute_orientation, absolute_frame.M);

  absolute_frame.p = abs_alpha_ * obj1.p + (1 - abs_alpha_) * obj2.p;

  return absolute_frame;
}

KDL::Frame ECTS::getRelativeMotionFrame(const KDL::Frame &obj1,
                                        const KDL::Frame &obj2) const
{
  KDL::Frame relative_frame;

  relative_frame.M = obj2.M;
  relative_frame.p = obj1.M * (obj2.p - obj1.p);

  return relative_frame;
}

void ECTS::getRelativeVelocity(const sensor_msgs::JointState &state,
                               const Vector3d &r1, const Vector3d &r2,
                               Vector6d &rel_vel) const
{
  KDL::Twist v1_eef, v2_eef;
  Vector6d v1_eef_eig, v2_eef_eig, v1, v2;
  Vector12d total_twist;
  Eigen::Matrix<double, 6, 12> relL = Eigen::Matrix<double, 6, 12>::Zero();
  Matrix12d W = computeW(r1, r2);
  kdl_manager_->getGrippingTwist(eef1_, state, v1_eef);
  kdl_manager_->getGrippingTwist(eef2_, state, v2_eef);

  tf::twistKDLToEigen(v1_eef, v1_eef_eig);
  tf::twistKDLToEigen(v2_eef, v2_eef_eig);

  total_twist.block<6, 1>(0, 0) = v1_eef_eig;
  total_twist.block<6, 1>(6, 0) = v2_eef_eig;

  relL.block<6, 6>(0, 0) = -Matrix6d::Identity();
  relL.block<6, 6>(0, 6) = Matrix6d::Identity();

  rel_vel = relL * W * total_twist;
}

Eigen::MatrixXd ECTS::computeJacobian(const sensor_msgs::JointState &state,
                                      const Vector3d &r1,
                                      const Vector3d &r2) const
{
  Matrix12d L = Matrix12d::Zero(), W = computeW(r1, r2);
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

  return J_e;
}
}  // namespace coordination_algorithms
