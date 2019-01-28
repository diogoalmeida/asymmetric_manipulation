#ifndef __ALGORITHM_BASE__
#define __ALGORITHM_BASE__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <limits>

namespace coordination_algorithms
{
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 12> MatrixRelLinkingd;
typedef Eigen::Matrix<double, 14, 6> MatrixInvRelativeJacd;
typedef Eigen::Matrix<double, 14, 12> MatrixInvECTSd;
typedef Eigen::Matrix<double, 14, 14> Matrix14d;

/**
  Implements an interface for the coordination algorithms
**/
class AlgorithmBase
{
 public:
  /**
    Construct a coordination algorithm.
  **/
  AlgorithmBase(const Vector3d &pos_upper_ct, const Vector3d &pos_lower_ct,
                double ori_ct);
  ~AlgorithmBase() {}

  /**
  Implement the coordination algorithm. Given reference absolute and relative
twists for a given task frame, solve the inverse differential kinematics to
obtain joint velocities for the two manipulators.

  @param state The dual-arm manipulator joint state.
  @param r1 The virtual stick connecting eef_1 to the task frame.
  @param r2 The virtual stick connecting eef_2 to the task frame.
  @param abs_twist The absolute motion twist in the task frame.
  @param rel_twist The relative motion twist in the task frame.
  @returns The resolved joint velocities for the two manipulators.
**/
  virtual Eigen::VectorXd control(const sensor_msgs::JointState &state,
                                  const Vector3d &r1, const Vector3d &r2,
                                  const Vector6d &abs_twist,
                                  const Vector6d &rel_twist) = 0;

  /**
    Returns a joint manipulability measure for the particular algorithm
  implementation.
  **/
  double getJointManipulability() const { return joint_manip_; };

  /**
    Allows derived classes to obtain the required data for their secundary
  tasks.

    @returns False if the required data is not obtained.
  **/
  virtual bool getSecundaryTask() { return true; };

  /**
    Sets the dynamic alpha mode. If the algorithm supports it, dynamic alpha
  sets the degree of cooperation of the arms according to some pre-defined
  logic.

    @param val If set to true, dynamic alpha will be activated. Set to false, it
  will not.
  **/
  void setDynamicAlpha(bool val) { dynamic_alpha_ = val; }

  /**
    Sets whether or not absolute limits will be considered in the ik solution.
  **/
  void setAbsoluteLimits(bool val) { use_absolute_limits_ = val; }

  /**
    Provides the algorithm with information on the absolute pose of the
  dual-armed system.
  **/
  void setAbsolutePose(const geometry_msgs::Pose &pose) { abs_pose_ = pose; }
  void setAlpha(double alpha) { alpha_ = alpha; }
  double getAlpha() const { return alpha_; }

  std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
  std::string eef1_, eef2_;
  std::string base_;

 protected:
  ros::NodeHandle nh_;
  bool dynamic_alpha_, use_absolute_limits_;
  double alpha_, damping_, joint_manip_;
  geometry_msgs::Pose abs_pose_;
  Vector6d abs_twist_;
  Vector3d pos_upper_ct_, pos_lower_ct_;
  double ori_ct_;
  double sec_pos_gain_, sec_ori_gain_;

  /**
    Initializes the parameters required to use an algorithm.

    @returns True in case of success, False otherwise.
  **/
  bool init();

  /**
    Sets up the information of an arm in the kdl_manager_.

    @param arm_name The key to be used in kdl_manager to refer to the arm.
    @param eef_name The arm's kinematic chain end-effector.
    @returns True in case of success, False otherwise.
  **/
  bool setArm(const std::string &arm_name, std::string &eef_name);

  /**
    Compute the wrench conversion matrix which maps twists in the task frames to
    eef twists.

    @param r1 Virtual stick connecting eef1 to its corresponding task frame.
    @param r2 Virtual stick connecting eef2 to its corresponding task frame.
  **/
  Matrix12d computeW(const Vector3d &r1, const Vector3d &r2) const;

  /**
    Compute the absolute motion task command, to be used as a potential
  secundary task.

    @param abs_pose The absolute pose of the system.
    @param abs_twist The absolute twist at the absolute frame.
  **/
  Vector6d computeAbsTask(const geometry_msgs::Pose &abs_pose) const;

  /**
    Compute the derivate of functions with domain in the joint space and return
  a real value.

    @param fun The function to compute the derivative for.
    @param state The joint state around which to estimate the derivative.
  **/
  double computeDerivative(std::function<double(sensor_msgs::JointState)> fun,
                           const sensor_msgs::JointState &state) const;
};
}  // namespace coordination_algorithms

#endif
