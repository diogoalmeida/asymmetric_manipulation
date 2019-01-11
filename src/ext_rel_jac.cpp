#include <coordination_algorithms/ext_rel_jac.hpp>

namespace coordination_algorithms
{
ExtRelJac::ExtRelJac() : AlgorithmBase() {}

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
  MatrixRelLinkingd L_asym = MatrixRelLinkingd::Zero(),
                    L_sim = MatrixRelLinkingd::Zero();
  KDL::Jacobian J1_kdl, J2_kdl;
  Eigen::MatrixXd J_sim, J_asym, J;

  kdl_manager_->getJacobian(eef1_, state, J1_kdl);
  kdl_manager_->getJacobian(eef2_, state, J2_kdl);

  J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
  J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
  J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

  alpha_ = computeAlpha(computeAbsolutePose(state), J1_kdl.data, J2_kdl.data);

  double scaling = 1 / ((1 - alpha_) * (1 - alpha_) + alpha_ * alpha_);
  L_asym.block<6, 6>(0, 0) = -(1 - alpha_) * scaling * Matrix6d::Identity();
  L_asym.block<6, 6>(0, 6) = alpha_ * scaling * Matrix6d::Identity();

  L_sim.block<6, 6>(0, 0) = -Matrix6d::Identity();
  L_sim.block<6, 6>(0, 6) = Matrix6d::Identity();

  J_asym = L_asym * W * J;
  J_sim = L_sim * W * J;

  MatrixInvRelativeJacd damped_sim_inverse =
      J_sim.transpose() *
      (J_sim * J_sim.transpose() + damping_ * Matrix6d::Identity()).inverse();
  MatrixInvRelativeJacd damped_asym_inverse =
      J_asym.transpose() *
      (J_asym * J_asym.transpose() + damping_ * Matrix6d::Identity()).inverse();

  Eigen::VectorXd qdot_asym = damped_asym_inverse * rel_twist;
  Eigen::VectorXd qdot_sym = damped_sim_inverse * rel_twist;

  return qdot_sym +
         (Matrix14d::Identity() - damped_sim_inverse * J_sim) * qdot_asym;
}

double ExtRelJac::computeAlpha(const geometry_msgs::Pose &abs_pose,
                               const Eigen::MatrixXd &J1,
                               const Eigen::MatrixXd &J2) const
{
  Vector6d pose_eig;
  Vector6d f_values;
  Eigen::Vector3d position_eig;
  Eigen::Quaterniond orientation_eig;

  tf::pointMsgToEigen(abs_pose.position, position_eig);
  tf::quaternionMsgToEigen(abs_pose.orientation, orientation_eig);
  pose_eig.block<3, 1>(0, 0) = position_eig;
  pose_eig.block<3, 1>(3, 0) =
      orientation_eig.toRotationMatrix().eulerAngles(0, 1, 2);

  double d = 0;
  for (unsigned int i = 0; i < 6; i++)
  {
    if (pose_eig[i] < pose_lower_thr_[i])
    {
      d = fabs(pose_lower_thr_[i] - pose_eig[i]) /
          fabs(pose_lower_thr_[i] - pose_lower_ct_[i]);
    }
    else if (pose_eig[i] > pose_upper_thr_[i])
    {
      d = fabs(pose_upper_thr_[i] - pose_eig[i]) /
          fabs(pose_upper_thr_[i] - pose_upper_ct_[i]);
    }
    else
    {
      f_values[i] = 0.0;
      continue;
    }

    f_values[i] = 1.5 * d * d - d * d * d;
  }

  double mu1, mu2;

  mu1 = sqrt((J1 * J1.transpose()).determinant());
  mu2 = sqrt((J2 * J2.transpose()).determinant());

  if (mu2 > mu1)
  {
    return 0.5 + f_values.maxCoeff();
  }

  return 0.5 - f_values.maxCoeff();
}
}  // namespace coordination_algorithms
