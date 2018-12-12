#include <coordination_algorithms/rel_jac.hpp>

namespace coordination_algorithms
{
RelJac::RelJac() : AlgorithmBase() {}

bool RelJac::init()
{
  if (!generic_control_toolbox::MatrixParser::parseMatrixData(
          Kp_, "secundary_gain", nh_))
  {
    return false;
  }

  // setup secundary markers over service call

  return true;
}

bool RelJac::getSecundaryTask()
{
  // use TF listener to get current pose for the secundary task. Return false if
  // TF fails.

  return true;
}

Eigen::VectorXd RelJac::control(const sensor_msgs::JointState &state,
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
  MatrixRelLinkingd L_sim = MatrixRelLinkingd::Zero();
  KDL::Jacobian J1_kdl, J2_kdl;
  Eigen::MatrixXd J_sim, J_sec, J;

  kdl_manager_->getJacobian(eef1_, state, J1_kdl);
  kdl_manager_->getJacobian(eef2_, state, J2_kdl);

  J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
  J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
  J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

  L_sim.block<6, 6>(0, 0) = -Matrix6d::Identity();
  L_sim.block<6, 6>(0, 6) = Matrix6d::Identity();

  J_sec = J;
  J_sec.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) =
      Eigen::MatrixXd::Zero(6, J2_kdl.columns());
  J_sim = L_sim * W * J;

  MatrixInvRelativeJacd damped_sim_inverse =
      J_sim.transpose() *
      (J_sim * J_sim.transpose() + damping_ * Matrix6d::Identity()).inverse();
  MatrixInvRelativeJacd damped_sec_inverse =
      J_sec.transpose() *
      (J_sec * J_sec.transpose() + damping_ * Matrix6d::Identity()).inverse();

  Eigen::VectorXd qdot_sec = damped_sec_inverse * sec_twist;
  Eigen::VectorXd qdot_sym = damped_sim_inverse * rel_twist;

  return qdot_sym +
         (Matrix14d::Identity() - damped_sim_inverse * J_sim) * qdot_sec;
}
}  // namespace coordination_algorithms
