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
      Implement the coordination algorithm. Given reference absolute and relative twists for a given task frame,
      solve the inverse differential kinematics to obtain joint velocities for the two manipulators.

      @param state The dual-arm manipulator joint state.
      @param r1 The virtual stick connecting eef_1 to the task frame.
      @param r2 The virtual stick connecting eef_2 to the task frame.
      @param abs_twist The absolute motion twist in the task frame.
      @param rel_twist The relative motion twist in the task frame.
      @returns The resolved joint velocities for the two manipulators.
    **/
    virtual Eigen::VectorXd control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &abs_twist, const Vector6d &rel_twist) = 0;

    /**
      Compute a rotation matrix which converts from the relative motion frame to the base frame.
      The coordinated frames are based on frames rigidly attached to the end-effectors and specified
      by the user.

      @param obj1 Object frame rigidly attached to eef1.
      @param obj2 Object frame rigidly attached to eef2.
    **/
    virtual Eigen::Matrix3d getRelativeToBase(const KDL::Frame &obj1, const KDL::Frame &obj2) const = 0;

    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    std::string eef1_, eef2_;
  protected:
    ros::NodeHandle nh_;
    std::string base_;

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
  };
}

#endif
