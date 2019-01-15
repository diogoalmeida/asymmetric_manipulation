#include <coordination_algorithms/ext_rel_jac.hpp>

namespace coordination_algorithms
{
ExtRelJac::ExtRelJac(const std::vector<double> &pos_upper_ct,
                     const std::vector<double> &pos_upper_thr,
                     const std::vector<double> &pos_lower_ct,
                     const std::vector<double> &pos_lower_thr,
                     const std::vector<double> &ori_ct,
                     const std::vector<double> &ori_thr)
    : AlgorithmBase(pos_upper_ct, pos_upper_thr, pos_lower_ct, pos_lower_thr,
                    ori_ct, ori_thr)
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
  MatrixRelLinkingd L_asym = MatrixRelLinkingd::Zero(),
                    L_sim = MatrixRelLinkingd::Zero();
  KDL::Jacobian J1_kdl, J2_kdl;
  Eigen::MatrixXd J_sim, J_asym, J;

  kdl_manager_->getJacobian(eef1_, state, J1_kdl);
  kdl_manager_->getJacobian(eef2_, state, J2_kdl);

  J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
  J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
  J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

  if (dynamic_alpha_)
  {
    alpha_ = computeAlpha(J1_kdl.data, J2_kdl.data);
  }

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

double ExtRelJac::computeAlpha(const Eigen::MatrixXd &J1,
                               const Eigen::MatrixXd &J2) const
{
  coordination_algorithms::Vector6d pose_eig;
  coordination_algorithms::Vector6d f_values;
  Eigen::Vector3d position_eig;
  Eigen::Quaterniond orientation_eig;
  Eigen::Affine3d abs_eig;

  tf::pointMsgToEigen(abs_pose_.position, position_eig);
  tf::poseMsgToEigen(abs_pose_, abs_eig);
  pose_eig.block<3, 1>(0, 0) = position_eig;
  pose_eig[3] =
      acos(abs_eig.matrix().block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitX()));
  pose_eig[4] =
      acos(abs_eig.matrix().block<3, 1>(0, 1).dot(Eigen::Vector3d::UnitY()));
  pose_eig[5] =
      acos(abs_eig.matrix().block<3, 1>(0, 2).dot(Eigen::Vector3d::UnitZ()));

  double d = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    d = 0;
    if (pose_eig[i] < pos_lower_thr_[i])
    {
      d = fabs(pos_lower_thr_[i] - pose_eig[i]) /
          fabs(pos_lower_thr_[i] - pos_lower_ct_[i]);
    }
    else if (pose_eig[i] > pos_upper_thr_[i])
    {
      d = fabs(pos_upper_thr_[i] - pose_eig[i]) /
          fabs(pos_upper_thr_[i] - pos_upper_ct_[i]);
    }

    f_values[i] = 1.5 * d * d - d * d * d;

    d = 0;
    if (pose_eig[i + 3] > ori_thr_[i])
    {
      d = fabs(ori_thr_[i] - pose_eig[i + 3]) / fabs(ori_thr_[i] - ori_ct_[i]);
    }

    f_values[i + 3] = 1.5 * d * d - d * d * d;
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
