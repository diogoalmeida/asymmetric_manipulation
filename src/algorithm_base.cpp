#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
AlgorithmBase::AlgorithmBase() : nh_("~"), alpha_(0.5), damping_(0.0001)
{
  if (!init())
  {
    throw std::logic_error("Missing parameters for the algorithm");
  }
}

bool AlgorithmBase::init()
{
  if (!nh_.getParam("kinematic_chain_base_link", base_))
  {
    ROS_ERROR("Missing kinematic_chain_base_link parameter");
    return false;
  }

  if (!nh_.getParam("absolute_pose_limits/upper_limits", pose_upper_ct_))
  {
    ROS_ERROR("Missing absolute_pose_limits/upper_limits parameter");
    return false;
  }

  if (!nh_.getParam("absolute_pose_limits/upper_thresholds", pose_upper_thr_))
  {
    ROS_ERROR("Missing absolute_pose_limits/upper_thresholds parameter");
    return false;
  }

  if (!nh_.getParam("absolute_pose_limits/lower_limits", pose_lower_ct_))
  {
    ROS_ERROR("Missing absolute_pose_limits/lower_limits parameter");
    return false;
  }

  if (!nh_.getParam("absolute_pose_limits/lower_thresholds", pose_lower_thr_))
  {
    ROS_ERROR("Missing absolute_pose_limits/lower_thresholds parameter");
    return false;
  }

  if (pose_upper_ct_.size() != 6 || pose_upper_thr_.size() != 6 ||
      pose_lower_ct_.size() != 6 || pose_lower_thr_.size() != 6)
  {
    ROS_ERROR("The absolute pose limits must all be vectors of length 6!");
    return false;
  }

  kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_);

  if (!setArm("eef1", eef1_))
  {
    return false;
  }

  if (!setArm("eef2", eef2_))
  {
    return false;
  }

  return true;
}

geometry_msgs::Pose AlgorithmBase::computeAbsolutePose(
    const sensor_msgs::JointState &state) const
{
  KDL::Frame p1, p2;
  Eigen::Affine3d p1_eig, p2_eig;

  kdl_manager_->getGrippingPoint(eef1_, state, p1);
  kdl_manager_->getGrippingPoint(eef2_, state, p2);

  Eigen::Vector3d avg_pos = (p1_eig.translation() + p2_eig.translation()) / 2;
  Eigen::Matrix3d r1 = p1_eig.matrix().block<3, 3>(0, 0);
  Eigen::Matrix3d r2 = p2_eig.matrix().block<3, 3>(0, 0);
  Eigen::Matrix3d rel = r1.transpose() * r2;
  Eigen::AngleAxisd rel_aa(rel);
  Eigen::AngleAxisd abs_aa(rel_aa.angle() / 2, rel_aa.axis());
  Eigen::Matrix3d r_abs = r1 * abs_aa.toRotationMatrix();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  T.block<3, 3>(0, 0) = r_abs;
  T.block<3, 1>(3, 0) = avg_pos;

  Eigen::Affine3d avg_eig(T);
  geometry_msgs::Pose p_avg;
  tf::poseEigenToMsg(avg_eig, p_avg);
  return p_avg;
}

bool AlgorithmBase::setArm(const std::string &arm_name, std::string &eef_name)
{
  generic_control_toolbox::ArmInfo info;

  if (!generic_control_toolbox::getArmInfo(arm_name, info))
  {
    return false;
  }

  eef_name = info.kdl_eef_frame;

  if (!generic_control_toolbox::setKDLManager(info, kdl_manager_))
  {
    return false;
  }

  return true;
}

Matrix12d AlgorithmBase::computeW(const Vector3d &r1, const Vector3d &r2) const
{
  Matrix12d W = Matrix12d::Identity();

  W.block<3, 3>(0, 3) =
      -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r1);
  W.block<3, 3>(6, 9) =
      -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r2);

  return W;
}
}  // namespace coordination_algorithms
