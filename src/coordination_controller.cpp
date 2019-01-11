#include <coordination_experiments/coordination_controller.hpp>

namespace coordination_experiments
{
CoordinationController::CoordinationController(const std::string &action_name)
    : ControllerTemplate<
          CoordinationControllerAction, CoordinationControllerGoal,
          CoordinationControllerFeedback, CoordinationControllerResult>(
          action_name)
{
  nh_ = ros::NodeHandle("~");

  if (!init())
  {
    throw std::logic_error(
        "Missing parameters for the coordination controller");
  }
}

void CoordinationController::twistCommandCb(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  tf::twistMsgToEigen(msg->twist, commanded_rel_twist_);
}

sensor_msgs::JointState CoordinationController::controlAlgorithm(
    const sensor_msgs::JointState &current_state, const ros::Duration &dt)
{
  sensor_msgs::JointState ret = current_state;
  Eigen::VectorXd q1, q2;
  double x, y, z, w;

  alg_->kdl_manager_->getJointPositions(alg_->eef1_, current_state, q1);
  alg_->kdl_manager_->getJointPositions(alg_->eef2_, current_state, q2);

  if (max_time_ > 0 && (ros::Time::now() - init_time_).toSec() > max_time_)
  {
    action_server_->setSucceeded();
  }

  if (newGoal_)
  {
    q1_init_ = q1;
    q2_init_ = q2;
    target_joint_positions_ =
        Eigen::MatrixXd::Zero(num_joints_[LEFT] + num_joints_[RIGHT], 1);
    target_joint_positions_.block(0, 0, num_joints_[LEFT], 1) = q1;
    target_joint_positions_.block(num_joints_[LEFT], 0, num_joints_[RIGHT], 1) =
        q2;

    newGoal_ = false;
    if (!alg_->getSecundaryTask())
    {
      action_server_->setAborted();
      return ret;
    }
  }

  Eigen::Matrix<double, 6, 1> rel_twist, abs_twist;
  Eigen::Vector3d r1, r2;

  if (control_type_ == align)
  {
    rel_twist = computeAlignRelativeTwist(current_state);
    computeAlignVirtualSticks(current_state, r1, r2);
  }
  else if (control_type_ == twist)
  {
    rel_twist = commanded_rel_twist_;
    computeTwistVirtualSticks(current_state, r1, r2);
  }

  abs_twist = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::VectorXd joint_velocities = alg_->controlAlgorithm(
      current_state, r1, r2, abs_twist, Kp_r_ * rel_twist);

  Eigen::Matrix<double, 6, 1> meas_abs_twist, meas_rel_twist;

  for (unsigned int i = 0; i < num_joints_[LEFT]; i++)
  {
    if (std::abs(target_joint_positions_(i) - q1(i)) < max_joint_pos_error_)
    {
      target_joint_positions_(i) += joint_velocities(i) * dt.toSec();
    }
  }

  for (unsigned int i = 0; i < num_joints_[RIGHT]; i++)
  {
    if (std::abs(target_joint_positions_(num_joints_[LEFT] + i) - q2(i)) <
        max_joint_pos_error_)
    {
      target_joint_positions_(num_joints_[LEFT] + i) +=
          joint_velocities(num_joints_[LEFT] + i) * dt.toSec();
    }
  }

  Eigen::VectorXd q_init_total =
                      Eigen::VectorXd::Zero(q1.rows() + q2.rows(), 1),
                  q_total = Eigen::VectorXd::Zero(q1.rows() + q2.rows(), 1);
  q_init_total.block(0, 0, q1.rows(), 1) = q1_init_;
  q_init_total.block(q1.rows(), 0, q2.rows(), 1) = q2_init_;
  q_total.block(0, 0, q1.rows(), 1) = q1;
  q_total.block(q1.rows(), 0, q2.rows(), 1) = q2;
  feedback_.joint_space_norm = (q_init_total - q_total).norm();

  alg_->kdl_manager_->getJointState(
      alg_->eef1_, target_joint_positions_.block(0, 0, num_joints_[LEFT], 1),
      joint_velocities.block(0, 0, num_joints_[LEFT], 1), ret);
  alg_->kdl_manager_->getJointState(
      alg_->eef2_,
      target_joint_positions_.block(num_joints_[LEFT], 0, num_joints_[RIGHT],
                                    1),
      joint_velocities.block(num_joints_[LEFT], 0, num_joints_[RIGHT], 1), ret);

  return ret;
}

Eigen::Matrix<double, 6, 1> CoordinationController::computeAlignRelativeTwist(
    const sensor_msgs::JointState &state)
{
  KDL::Frame p1, obj1, p2, obj2;
  KDL::Vector rel_perr;
  alg_->kdl_manager_->getGrippingPoint(alg_->eef1_, state, p1);
  alg_->kdl_manager_->getGrippingPoint(alg_->eef2_, state, p2);

  // Obtain the object frames in the base frame
  obj1 = p1 * obj_in_eef_[LEFT];
  obj2 = p2 * obj_in_eef_[RIGHT];

  Eigen::Affine3d obj1_eig, obj2_eig, relative_frame;
  tf::transformKDLToEigen(obj1, obj1_eig);
  tf::transformKDLToEigen(obj2, obj2_eig);
  Eigen::Matrix3d relative_error =
      obj1_eig.linear().transpose() *
      obj2_eig.linear();  // Orientation error between the two object frames.
  Eigen::Quaterniond quat_err(relative_error);
  Eigen::AngleAxisd relative_error_ang_axis(quat_err);

  // Compute desired relative twist
  Eigen::Matrix<double, 6, 1> rel_twist, abs_twist;
  rel_perr = (-obj2.p + obj1.p);
  rel_twist.block<3, 1>(0, 0) << rel_perr.x(), rel_perr.y(), rel_perr.z();
  rel_twist.block<3, 1>(3, 0) =
      obj1_eig.linear() *
      quat_err.inverse().vec();  // The error between frames needs to be
                                 // converted to the base frame.

  feedback_.relative_error_angle = relative_error_ang_axis.angle();
  feedback_.relative_error_norm = rel_perr.Norm();
  feedback_.curr_alpha = alg_->getAlpha();
  tf::pointKDLToMsg(rel_perr, feedback_.relative_error);

  return rel_twist;
}

void CoordinationController::computeAlignVirtualSticks(
    const sensor_msgs::JointState &state, Eigen::Vector3d &r1,
    Eigen::Vector3d &r2)
{
  KDL::Frame p1, obj1, p2, obj2;

  alg_->kdl_manager_->getGrippingPoint(alg_->eef1_, state, p1);
  alg_->kdl_manager_->getGrippingPoint(alg_->eef2_, state, p2);

  // Obtain the object frames in the base frame
  obj1 = p1 * obj_in_eef_[LEFT];
  obj2 = p2 * obj_in_eef_[RIGHT];
  tf::vectorKDLToEigen(obj1.p - p1.p, r1);
  tf::vectorKDLToEigen(obj2.p - p2.p, r2);
}

void CoordinationController::computeTwistVirtualSticks(
    const sensor_msgs::JointState &state, Eigen::Vector3d &r1,
    Eigen::Vector3d &r2)
{
  KDL::Frame p1, p2, obj1;

  alg_->kdl_manager_->getGrippingPoint(alg_->eef1_, state, p1);
  alg_->kdl_manager_->getGrippingPoint(alg_->eef2_, state, p2);

  obj1 = p1 * obj_in_eef_[LEFT];

  tf::vectorKDLToEigen(obj1.p - p1.p, r1);
  tf::vectorKDLToEigen(obj1.p - p2.p, r2);
}

bool CoordinationController::parseGoal(
    boost::shared_ptr<const CoordinationControllerGoal> goal)
{
  std_srvs::Empty srv;

  if (goal->control_mode.controller == goal->control_mode.RESET)
  {
    if (reset_client_.call(srv))
    {
      action_server_->setSucceeded();
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (goal->control_mode.controller == goal->control_mode.ECTS)
  {
    if (!reset_client_.call(srv))
    {
      return false;
    }

    alg_ = std::make_shared<coordination_algorithms::ECTS>();
  }
  else if (goal->control_mode.controller == goal->control_mode.EXTRELJAC)
  {
    if (!reset_client_.call(srv))
    {
      return false;
    }

    alg_ = std::make_shared<coordination_algorithms::ExtRelJac>();
  }
  else if (goal->control_mode.controller == goal->control_mode.RELJAC)
  {
    if (!reset_client_.call(srv))
    {
      return false;
    }

    alg_ = std::make_shared<coordination_algorithms::RelJac>();
  }
  else
  {
    ROS_ERROR_STREAM("Unknown controller: " << goal->control_mode.controller);
    return false;
  }

  ros::Duration(0.1)
      .sleep();  // let the controller get the updated simulation joint state
  ros::spinOnce();

  if (goal->alpha < 0 || goal->alpha > 1)
  {
    ROS_ERROR("Alpha must be between 0 and 1");
    return false;
  }

  alg_->setAlpha(goal->alpha);

  if (!alg_->kdl_manager_->getNumJoints(alg_->eef1_, num_joints_[LEFT]))
  {
    return false;
  }

  if (!alg_->kdl_manager_->getNumJoints(alg_->eef2_, num_joints_[RIGHT]))
  {
    return false;
  }

  if (goal->input_mode.mode == goal->input_mode.ALIGN_FRAMES)
  {
    control_type_ = align;
  }
  else
  {
    control_type_ = twist;

    twist_sub_ = nh_.subscribe(goal->input_mode.twist_topic, 1,
                               &CoordinationController::twistCommandCb, this);
  }

  if (!initializeObjectFrames())  // if control_type_ is twist we use obj1 as
                                  // C-Frame
  {
    return false;
  }

  if (goal->max_time > 0)
  {
    max_time_ = goal->max_time;
  }
  else
  {
    max_time_ = -1;
  }

  ROS_INFO("Coordination controller received a valid goal!");
  newGoal_ = true;
  feedback_.joint_space_norm = 0.0;
  init_time_ = ros::Time::now();
  return true;
}

bool CoordinationController::initializeObjectFrames()
{
  geometry_msgs::PoseStamped object_pose;
  object_pose.header.frame_id = obj_frame_[LEFT];
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
      listener_.transformPose(eef_frame_[LEFT], object_pose, object_pose);
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
    ROS_ERROR_STREAM("Failed to obtain pose of " << obj_frame_[LEFT] << " in "
                                                 << eef_frame_[LEFT]);
    return false;
  }
  tf::poseMsgToKDL(object_pose.pose, obj_in_eef_[LEFT]);

  object_pose.header.frame_id = obj_frame_[RIGHT];
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
      listener_.transformPose(eef_frame_[RIGHT], object_pose, object_pose);
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
    ROS_ERROR_STREAM("Failed to obtain pose of " << obj_frame_[RIGHT] << " in "
                                                 << eef_frame_[RIGHT]);
    return false;
  }
  tf::poseMsgToKDL(object_pose.pose, obj_in_eef_[RIGHT]);

  return true;
}

void CoordinationController::resetController() { newGoal_ = true; }

bool CoordinationController::init()
{
  if (!nh_.getParam("max_tf_attempts", max_tf_attempts_))
  {
    ROS_ERROR("Missing max_tf_attemps parameter.");
    return false;
  }

  if (!nh_.getParam("max_joint_pos_error", max_joint_pos_error_))
  {
    ROS_ERROR("Missing max_joint_pos_error parameter");
    return false;
  }

  if (!nh_.getParam("eef1/kdl_eef_frame", eef_frame_[LEFT]))
  {
    ROS_ERROR("Missing left_eef_frame parameter");
    return false;
  }

  if (!nh_.getParam("left_obj_frame", obj_frame_[LEFT]))
  {
    ROS_ERROR("Missing left_obj_frame parameter");
    return false;
  }

  if (!nh_.getParam("eef2/kdl_eef_frame", eef_frame_[RIGHT]))
  {
    ROS_ERROR("Missing right_eef_frame parameter");
    return false;
  }

  if (!nh_.getParam("right_obj_frame", obj_frame_[RIGHT]))
  {
    ROS_ERROR("Missing right_obj_frame parameter");
    return false;
  }

  if (!generic_control_toolbox::MatrixParser::parseMatrixData(
          Kp_r_, "relative_gain", nh_))
  {
    return false;
  }

  if (Kp_r_.cols() != 6)
  {
    ROS_ERROR("Invalid relative gain dimensions. Must be 6x6");
    return false;
  }

  reset_client_ = nh_.serviceClient<std_srvs::Empty>("/state_reset");

  newGoal_ = true;
  return true;
}
}  // namespace coordination_experiments

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordination_controller");
  coordination_experiments::CoordinationController controller(
      "coordination_control");
  generic_control_toolbox::ControllerActionNode action_node;

  action_node.runController(controller);
  return 0;
}
