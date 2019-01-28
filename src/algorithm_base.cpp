#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
AlgorithmBase::AlgorithmBase(const Vector3d &pos_upper_ct,
                             const Vector3d &pos_lower_ct, double ori_ct)
    : nh_("~"),
      alpha_(0.5),
      damping_(0.0001),
      dynamic_alpha_(false),
      pos_upper_ct_(pos_upper_ct),
      pos_lower_ct_(pos_lower_ct),
      ori_ct_(ori_ct),
      joint_manip_(0.0)
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

Vector6d AlgorithmBase::computeAbsTask(
    const geometry_msgs::Pose &abs_pose) const
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

double AlgorithmBase::computeDerivative(
    std::function<double(sensor_msgs::JointState)> fun,
    const sensor_msgs::JointState &state) const
{
  unsigned int n1, n2;
  double h = std::cbrt(std::numeric_limits<double>::min());

  kdl_manager_->getNumJoints(eef1_, n1);
  kdl_manager_->getNumJoints(eef2_, n2);
  KDL::JntArray q1, q2;

  kdl_manager_->getJointPositions(eef1_, state, q1);
  kdl_manager_->getJointPositions(eef2_, state, q2);

  Eigen::VectorXd q1u, q1l, q2u, q2l;

  q1u = q1.data + h * Eigen::VectorXd::Ones(n1, 1);
  q1l = q1.data - h * Eigen::VectorXd::Ones(n1, 1);
  q2u = q2.data + h * Eigen::VectorXd::Ones(n2, 1);
  q2l = q2.data - h * Eigen::VectorXd::Ones(n2, 1);

  sensor_msgs::JointState state_up = state, state_down = state;
  kdl_manager_->getJointState(eef1_, q1u, Eigen::VectorXd::Zero(n1, 1),
                              state_up);
  kdl_manager_->getJointState(eef2_, q2u, Eigen::VectorXd::Zero(n2, 1),
                              state_up);
  kdl_manager_->getJointState(eef1_, q1l, Eigen::VectorXd::Zero(n1, 1),
                              state_down);
  kdl_manager_->getJointState(eef2_, q2l, Eigen::VectorXd::Zero(n2, 1),
                              state_down);

  return (fun(state_up) - fun(state_down)) / (2 * h);
}
}  // namespace coordination_algorithms
