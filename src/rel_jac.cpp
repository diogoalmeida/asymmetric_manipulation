#include <coordination_algorithms/rel_jac.hpp>

namespace coordination_algorithms
{
RelJac::RelJac(const Vector3d &pos_upper_ct, const Vector3d &pos_lower_ct,
               double pos_thr, double ori_ct, double ori_thr)
    : AlgorithmBase(pos_upper_ct, pos_lower_ct, pos_thr, ori_ct, ori_thr)
{
  if (!init())
  {
    throw std::logic_error(
        "Failed to initialize the relative Jacobian controller!");
  }
}

bool RelJac::init()
{
  if (!generic_control_toolbox::MatrixParser::parseMatrixData(
          Kp_, "secundary_gain", nh_))
  {
    return false;
  }

  return true;
}

bool RelJac::getSecundaryTask()
{
  geometry_msgs::PoseStamped task_pose;
  task_pose.header.frame_id = "secundary_pose";
  task_pose.header.stamp = ros::Time(0);
  task_pose.pose.position.x = 0;
  task_pose.pose.position.y = 0;
  task_pose.pose.position.z = 0;
  task_pose.pose.orientation.x = 0;
  task_pose.pose.orientation.y = 0;
  task_pose.pose.orientation.z = 0;
  task_pose.pose.orientation.w = 1;

  int attempts;
  for (attempts = 0; attempts < 5; attempts++)
  {
    try
    {
      listener_.transformPose("base", task_pose, task_pose);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("TF exception in coordination controller: %s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }

  if (attempts >= 5)
  {
    ROS_ERROR_STREAM("Failed to obtain pose of eef_target in base_link");
    return false;
  }

  tf::poseMsgToEigen(task_pose.pose, secundary_target_);

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

  joint_manip_ = std::sqrt((J_sim * J_sim.transpose()).determinant());

  MatrixInvRelativeJacd damped_sim_inverse =
      J_sim.transpose() *
      (J_sim * J_sim.transpose() + damping_ * Matrix6d::Identity()).inverse();
  MatrixInvECTSd damped_sec_inverse =
      J_sec.transpose() *
      (J_sec * J_sec.transpose() + damping_ * Matrix12d::Identity()).inverse();

  // compute twist from secundary task
  Eigen::Matrix3d orientation_err =
      secundary_target_.linear().transpose() * p1_eig.linear();
  Eigen::Quaterniond quat_err(orientation_err);
  Vector12d sec_twist = Vector12d::Zero();
  sec_twist.block<3, 1>(0, 0) =
      -p1_eig.translation() + secundary_target_.translation();
  sec_twist.block<3, 1>(3, 0) =
      secundary_target_.linear() * quat_err.inverse().vec();

  sec_twist.block<6, 1>(0, 0) = Kp_ * sec_twist.block<6, 1>(0, 0);
  Eigen::VectorXd qdot_sec = damped_sec_inverse * sec_twist;
  Eigen::VectorXd qdot_sym = damped_sim_inverse * rel_twist;

  return qdot_sym +
         (Matrix14d::Identity() - damped_sim_inverse * J_sim) * qdot_sec;
}
}  // namespace coordination_algorithms
