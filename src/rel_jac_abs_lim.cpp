#include <coordination_algorithms/rel_jac_abs_lim.hpp>

namespace coordination_algorithms
{
RelJacAbsLim::RelJacAbsLim() : AlgorithmBase()
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
  if (!nh_.getParam("absolute_position_limits/upper_limits", pos_upper_ct_))
  {
    ROS_ERROR("Missing absolute_position_limits/upper_limits parameter");
    return false;
  }

  if (!nh_.getParam("absolute_position_limits/upper_thresholds",
                    pos_upper_thr_))
  {
    ROS_ERROR("Missing absolute_position_limits/upper_thresholds parameter");
    return false;
  }

  if (!nh_.getParam("absolute_position_limits/lower_limits", pos_lower_ct_))
  {
    ROS_ERROR("Missing absolute_position_limits/lower_limits parameter");
    return false;
  }

  if (!nh_.getParam("absolute_position_limits/lower_thresholds",
                    pos_lower_thr_))
  {
    ROS_ERROR("Missing absolute_position_limits/lower_thresholds parameter");
    return false;
  }

  if (pos_upper_ct_.size() != 3 || pos_upper_thr_.size() != 3 ||
      pos_lower_ct_.size() != 3 || pos_lower_thr_.size() != 3)
  {
    ROS_ERROR("The absolute position limits must all be vectors of length 3!");
    return false;
  }

  if (!nh_.getParam("left_obj_frame", obj_frame_[LEFT]))
  {
    ROS_ERROR("Missing left_obj_frame parameter");
    return false;
  }

  if (!nh_.getParam("right_obj_frame", obj_frame_[RIGHT]))
  {
    ROS_ERROR("Missing right_obj_frame parameter");
    return false;
  }

  if (!nh_.getParam("eef1/kdl_eef_frame", eef_frame_[LEFT]))
  {
    ROS_ERROR("Missing eef1/kdl_eef_frame parameter");
    return false;
  }

  if (!nh_.getParam("eef2/kdl_eef_frame", eef_frame_[RIGHT]))
  {
    ROS_ERROR("Missing eef2/kdl_eef_frame parameter");
    return false;
  }

  if (!nh_.getParam("max_tf_attempts", max_tf_attempts_))
  {
    ROS_ERROR("Missing max_tf_attempts parameter");
    return false;
  }

  return true;
}

bool RelJacAbsLim::getSecundaryTask()
{
  geometry_msgs::PoseStamped object_pose;
  object_pose.header.frame_id = obj_frame_.at(LEFT);
  object_pose.header.stamp = ros::Time(0);
  object_pose.pose.position.x = 0;
  object_pose.pose.position.y = 0;
  object_pose.pose.position.z = 0;
  object_pose.pose.orientation.x = 0;
  object_pose.pose.orientation.y = 0;
  object_pose.pose.orientation.z = 0;
  object_pose.pose.orientation.w = 1;

  int attempts;
  for (attempts = 0; attempts < max_tf_attempts_; attempts++)
  {
    try
    {
      listener_.transformPose(eef_frame_.at(LEFT), object_pose, object_pose);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("TF exception in coordination controller: %s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }

  if (attempts >= max_tf_attempts_)
  {
    ROS_ERROR_STREAM("Failed to obtain pose of "
                     << obj_frame_.at(LEFT) << " in " << eef_frame_.at(LEFT));
    return false;
  }

  tf::poseMsgToKDL(object_pose.pose, obj_in_eef_[LEFT]);

  object_pose.header.frame_id = obj_frame_.at(RIGHT);
  object_pose.header.stamp = ros::Time(0);
  object_pose.pose.position.x = 0;
  object_pose.pose.position.y = 0;
  object_pose.pose.position.z = 0;
  object_pose.pose.orientation.x = 0;
  object_pose.pose.orientation.y = 0;
  object_pose.pose.orientation.z = 0;
  object_pose.pose.orientation.w = 1;

  for (attempts = 0; attempts < max_tf_attempts_; attempts++)
  {
    try
    {
      listener_.transformPose(eef_frame_.at(RIGHT), object_pose, object_pose);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("TF exception in coordination controller: %s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }

  if (attempts >= max_tf_attempts_)
  {
    ROS_ERROR_STREAM("Failed to obtain pose of "
                     << obj_frame_.at(RIGHT) << " in " << eef_frame_.at(RIGHT));
    return false;
  }
  tf::poseMsgToKDL(object_pose.pose, obj_in_eef_[RIGHT]);

  return true;
}

Eigen::VectorXd RelJacAbsLim::control(const sensor_msgs::JointState &state,
                                      const Vector3d &r1, const Vector3d &r2,
                                      const Vector6d &abs_twist,
                                      const Vector6d &rel_twist)
{
  KDL::Frame p1, p2, eef1, eef2, obj1, obj2;
  Eigen::Affine3d p1_eig, p2_eig, eef1_eig, eef2_eig;

  kdl_manager_->getGrippingPoint(eef1_, state, p1);
  kdl_manager_->getGrippingPoint(eef2_, state, p2);
  kdl_manager_->getEefPose(eef1_, state, eef1);
  kdl_manager_->getEefPose(eef2_, state, eef2);

  obj1 = p1 * obj_in_eef_.at(LEFT);
  obj2 = p2 * obj_in_eef_.at(RIGHT);

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
  MatrixInvRelativeJacd damped_sec_inverse =
      J_sec.transpose() *
      (J_sec * J_sec.transpose() + damping_ * Matrix6d::Identity()).inverse();

  // compute twist from secundary task
  geometry_msgs::Pose abs_pose;
  tf::pointKDLToMsg((obj1.p + obj2.p) / 2, abs_pose.position);
  Eigen::Quaterniond ori1, ori2;
  tf::quaternionKDLToEigen(obj1.M, ori1);
  tf::quaternionKDLToEigen(obj2.M, ori2);
  Eigen::Matrix3d ori1m = ori1.toRotationMatrix(),
                  ori2m = ori2.toRotationMatrix();
  Eigen::Matrix3d orirel = ori1m.transpose() * ori2m;
  Eigen::AngleAxisd orirel_aa(orirel);
  Eigen::AngleAxisd oriabs_aa(orirel_aa.angle() / 2, orirel_aa.axis());
  Eigen::Quaterniond q(oriabs_aa);
  tf::quaternionEigenToMsg(q, abs_pose.orientation);
  Vector6d sec_twist = computeAbsTask(abs_pose);

  Eigen::VectorXd qdot_sec = damped_sec_inverse * sec_twist;
  Eigen::VectorXd qdot_sym = damped_sim_inverse * rel_twist;

  return qdot_sym +
         (Matrix14d::Identity() - damped_sim_inverse * J_sim) * qdot_sec;
}

Vector6d RelJacAbsLim::computeAbsTask(const geometry_msgs::Pose &abs_pose) const
{
  Vector6d ret = Vector6d::Zero();
  Vector3d pos;

  tf::pointMsgToEigen(abs_pose.position, pos);

  for (unsigned int i = 0; i < 3; i++)
  {
    if (pos[i] > pos_upper_thr_[i])
    {
      ret[i] += 0.01 / (pos[i] - pos_upper_ct_[i]) +
                0.01 / (pos_upper_thr_[i] - pos_upper_ct_[i]);
    }

    if (pos[i] < pos_lower_thr_[i])
    {
      ret[i] += 0.01 / (pos[i] - pos_lower_ct_[i]) +
                0.01 / (pos_lower_thr_[i] - pos_upper_ct_[i]);
    }
  }

  return ret;
}
}  // namespace coordination_algorithms
