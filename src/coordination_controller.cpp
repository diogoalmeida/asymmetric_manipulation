#include <coordination_experiments/coordination_controller.hpp>

namespace coordination_experiments
{
  CoordinationController::CoordinationController(const std::string &action_name) : ControllerTemplate<CoordinationControllerAction,
                     CoordinationControllerGoal,
                     CoordinationControllerFeedback,
                     CoordinationControllerResult>(action_name)
  {
    nh_ = ros::NodeHandle("~");

    if (!init())
    {
      throw std::logic_error("Missing parameters for the coordination controller");
    }
  }

  sensor_msgs::JointState CoordinationController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    KDL::Frame p1, obj1, p2, obj2;
    KDL::Vector rel_perr, abs_perr;
    double x, y, z, w;
    alg_->kdl_manager_->getGrippingPoint(alg_->eef1_, current_state, p1);
    alg_->kdl_manager_->getGrippingPoint(alg_->eef2_, current_state, p2);

    // Obtain the object frames in the base frame
    obj1 = p1*obj_in_eef_[LEFT];
    obj2 = p2*obj_in_eef_[RIGHT];

    // Compute the angular component of the relative twist: orientation error in quaternion form, angular error
    // will be the vector part of the error, converted to the relative frame
    obj1.M.GetQuaternion(x, y, z, w);
    Eigen::Quaterniond obj1_rot(w, x, y, z);
    obj2.M.GetQuaternion(x, y, z, w);
    Eigen::Quaterniond obj2_rot(w, x, y, z);

    Eigen::Quaterniond quat_err = obj1_rot.inverse()*obj2_rot; // quaternion error
    Eigen::Affine3d obj1_eig, obj2_eig, relative_frame;
    tf::transformKDLToEigen(obj1, obj1_eig);
    tf::transformKDLToEigen(obj2, obj2_eig);
    Eigen::Matrix3d relative_orientation = obj1_eig.linear().transpose()*obj2_eig.linear();

    // Compute desired relative twist
    Eigen::Matrix<double, 6, 1> rel_twist, abs_twist;
    rel_perr = obj2.p - obj1.p;
    rel_twist.block<3,1>(0,0) << rel_perr.x(), rel_perr.y(), rel_perr.z();
    rel_twist.block<3,1>(3,0) = relative_orientation * quat_err.vec();

    Eigen::Vector3d r1, r2;
    abs_twist = Eigen::Matrix<double, 6, 1>::Zero();
    tf::vectorKDLToEigen(obj1.p - p1.p, r1);
    tf::vectorKDLToEigen(obj2.p - p2.p, r2);
    Eigen::VectorXd joint_velocities = alg_->control(current_state, r1, r2, abs_twist, rel_twist);
    alg_->kdl_manager_->getJointState(alg_->eef1_, joint_velocities.block(0, 0, num_joints_[LEFT], 0), ret);
    alg_->kdl_manager_->getJointState(alg_->eef2_, joint_velocities.block(0, 0, num_joints_[RIGHT], 0), ret);

    return ret;
  }

  bool CoordinationController::parseGoal(boost::shared_ptr<const CoordinationControllerGoal> goal)
  {
    if (goal->controller == goal->ECTS)
    {
      alg_ = std::make_shared<coordination_algorithms::ECTS>();
    }
    else
    {
      ROS_ERROR_STREAM("Unknown controller: " << goal->controller);
      return false;
    }

    if (!alg_->kdl_manager_->getNumJoints(alg_->eef1_, num_joints_[LEFT]))
    {
      return false;
    }

    if (!alg_->kdl_manager_->getNumJoints(alg_->eef2_, num_joints_[RIGHT]))
    {
      return false;
    }

    return true;
  }

  void CoordinationController::resetController()
  {

  }

  bool CoordinationController::init()
  {
    int max_tf_attempts;

    if (!nh_.getParam("max_tf_attempts", max_tf_attempts))
    {
      ROS_ERROR("Missing max_tf_attemps parameter.");
      return false;
    }

    geometry_msgs::PoseStamped object_pose;
    object_pose.header.frame_id = eef_frame_[LEFT];
    object_pose.header.stamp = ros::Time(0);
    object_pose.pose.position.x = 0;
    object_pose.pose.position.y = 0;
    object_pose.pose.position.z = 0;
    object_pose.pose.orientation.x = 0;
    object_pose.pose.orientation.y = 0;
    object_pose.pose.orientation.z = 0;
    object_pose.pose.orientation.w = 1;

    int attempts;
    for (attempts = 0; attempts < max_tf_attempts; attempts++)
    {
      try
      {
        listener_.transformPose(eef_frame_[LEFT], object_pose, object_pose);
        break;
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("TF exception in coordination controller: %s", ex.what());
        ros::Duration(0.01).sleep();
      }
    }

    if (attempts >= max_tf_attempts)
    {
      ROS_ERROR_STREAM("Failed to obtain pose of " << obj_frame_[LEFT] << " in " << eef_frame_[LEFT]);
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

    for (attempts = 0; attempts < max_tf_attempts; attempts++)
    {
      try
      {
        listener_.transformPose(eef_frame_[RIGHT], object_pose, object_pose);
        break;
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("TF exception in coordination controller: %s", ex.what());
        ros::Duration(0.01).sleep();
      }
    }

    if (attempts >= max_tf_attempts)
    {
      ROS_ERROR_STREAM("Failed to obtain pose of " << obj_frame_[RIGHT] << " in " << eef_frame_[RIGHT]);
      return false;
    }
    tf::poseMsgToKDL(object_pose.pose, obj_in_eef_[RIGHT]);

    return true;
  }
}

int main(int argc, char **argv)
{

}
