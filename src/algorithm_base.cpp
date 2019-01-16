#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
AlgorithmBase::AlgorithmBase(const Vector3d &pos_upper_ct,
                             const Vector3d &pos_lower_ct, double pos_thr,
                             double ori_ct, double ori_thr)
    : nh_("~"),
      alpha_(0.5),
      damping_(0.0001),
      dynamic_alpha_(false),
      pos_upper_ct_(pos_upper_ct),
      pos_lower_ct_(pos_lower_ct),
      pos_thr_(pos_thr),
      ori_ct_(ori_ct),
      ori_thr_(ori_thr)
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

  kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_);

  if (!setArm("eef1", eef1_))
  {
    return false;
  }

  if (!setArm("eef2", eef2_))
  {
    return false;
  }

  Eigen::Affine3d init_abs_pose = Eigen::Affine3d::Identity();
  tf::poseEigenToMsg(init_abs_pose, abs_pose_);

  return true;
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
