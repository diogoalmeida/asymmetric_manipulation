#include <coordination_algorithms/rel_jac_abs_lim.hpp>

namespace coordination_algorithms
{
RelJacAbsLim::RelJacAbsLim(const Vector3d &pos_upper_ct,
                           const Vector3d &pos_lower_ct, double pos_thr,
                           double ori_ct, double ori_thr)
    : AlgorithmBase(pos_upper_ct, pos_lower_ct, pos_thr, ori_ct, ori_thr),
      symmetric_(false)
{
  if (!init())
  {
    throw std::logic_error(
        "Failed to initialize the relative Jacobian controller with absolute "
        "motion limits!");
  }
}

bool RelJacAbsLim::init()
{
  if (!nh_.getParam("secundary_absolute_motion_task/position_gain",
                    sec_pos_gain_))
  {
    ROS_ERROR("Missing secundary_absolute_motion_task/position_gain parameter");
    return false;
  }

  if (!nh_.getParam("secundary_absolute_motion_task/orientation_gain",
                    sec_ori_gain_))
  {
    ROS_ERROR(
        "Missing secundary_absolute_motion_task/orientation_gain parameter");
    return false;
  }

  return true;
}

Eigen::VectorXd RelJacAbsLim::control(const sensor_msgs::JointState &state,
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
  MatrixRelLinkingd L_sim = MatrixRelLinkingd::Zero(),
                    L_abs = MatrixRelLinkingd::Zero();
  KDL::Jacobian J1_kdl, J2_kdl;
  Eigen::MatrixXd J_sim, J_sec, J;

  kdl_manager_->getJacobian(eef1_, state, J1_kdl);
  kdl_manager_->getJacobian(eef2_, state, J2_kdl);

  J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
  J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
  J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

  L_sim.block<6, 6>(0, 0) = -Matrix6d::Identity();
  L_sim.block<6, 6>(0, 6) = Matrix6d::Identity();
  L_abs.block<6, 6>(0, 0) = 0.5 * Matrix6d::Identity();
  L_abs.block<6, 6>(0, 6) = 0.5 * Matrix6d::Identity();

  J_sec = L_abs * W * J;
  J_sim = L_sim * W * J;

  MatrixInvRelativeJacd damped_sim_inverse =
      J_sim.transpose() *
      (J_sim * J_sim.transpose() + damping_ * Matrix6d::Identity()).inverse();

  // compute twist from secundary task
  Vector6d sec_twist = computeAbsTask(abs_pose_);
  Eigen::MatrixXd full_sec_twist;

  Eigen::MatrixXd damped_sec_inverse;
  if (symmetric_)
  {
    full_sec_twist = sec_twist;
    damped_sec_inverse =
        J_sec.transpose() *
        (J_sec * J_sec.transpose() + damping_ * Matrix6d::Identity()).inverse();
  }
  else
  {
    full_sec_twist = Eigen::Matrix<double, 12, 1>::Zero();
    full_sec_twist.block<6, 1>(0, 0) = sec_twist;
    full_sec_twist.block<6, 1>(6, 0) = sec_twist;
    damped_sec_inverse =
        J.transpose() *
        (J * J.transpose() + damping_ * Matrix12d::Identity()).inverse();
  }

  Eigen::VectorXd qdot_sec = damped_sec_inverse * full_sec_twist;
  Eigen::VectorXd qdot_sym = damped_sim_inverse * rel_twist;

  return qdot_sym +
         (Matrix14d::Identity() - damped_sim_inverse * J_sim) * qdot_sec;
}

Vector6d RelJacAbsLim::computeAbsTask(const geometry_msgs::Pose &abs_pose) const
{
  Vector6d ret = Vector6d::Zero();
  Vector3d pos;
  Eigen::Quaterniond absq;

  tf::pointMsgToEigen(abs_pose.position, pos);
  tf::quaternionMsgToEigen(abs_pose.orientation, absq);

  Eigen::AngleAxisd absaa(absq);

  for (unsigned int i = 0; i < 3; i++)
  {
    if (pos_upper_ct_[i] - pos[i] < 0)
    {
      ret[i] = sec_pos_gain_ * (pos_upper_ct_[i] - pos[i]);
    }

    if (pos[i] - pos_lower_ct_[i] < 0)
    {
      ret[i] = sec_pos_gain_ * (pos_lower_ct_[i] - pos[i]);
    }
  }

  if (fabs(absaa.angle()) >= ori_ct_)
  {
    Eigen::AngleAxisd target_aa(ori_ct_, absaa.axis());
    Eigen::Matrix3d rel_error =
        target_aa.toRotationMatrix().transpose() * absaa.toRotationMatrix();
    Eigen::Quaterniond relq(rel_error);
    ret.block<3, 1>(3, 0) =
        sec_ori_gain_ * target_aa.toRotationMatrix() * relq.inverse().vec();
  }

  return ret;
}
}  // namespace coordination_algorithms
