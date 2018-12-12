#ifndef __ALGORITHM_BASE__
#define __ALGORITHM_BASE__

#include <ros/ros.h>
#include <generic_control_toolbox/kdl_manager.hpp>

namespace coordination_algorithms
{
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 12> MatrixRelLinkingd;
typedef Eigen::Matrix<double, 14, 6> MatrixInvRelativeJacd;
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
  AlgorithmBase();
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

  void setRelativeAlpha(double alpha) { rel_alpha_ = alpha; }
  void setAbsoluteAlpha(double alpha) { abs_alpha_ = alpha; }
  double getRelativeAlpha() const { return rel_alpha_; }
  double getAbsoluteAlpha() const { return abs_alpha_; }

  std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
  std::string eef1_, eef2_;

 protected:
  ros::NodeHandle nh_;
  std::string base_;
  double abs_alpha_, rel_alpha_, damping_;

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
};
}  // namespace coordination_algorithms

#endif
