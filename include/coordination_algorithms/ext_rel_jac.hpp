#ifndef __EXTENDED_RELATIVE_JACOBIAN__
#define __EXTENDED_RELATIVE_JACOBIAN__

#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
  /**
    Implements the extended relative Jacobian algorithm
  **/
  class ExtRelJac : public AlgorithmBase
  {
  public:
    ExtRelJac();
    ~ExtRelJac() {}

    Eigen::VectorXd control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &abs_twist, const Vector6d &rel_twist);
    Eigen::Matrix3d getRelativeToBase(const KDL::Frame &obj1, const KDL::Frame &obj2) const;

  private:
    /**
      Computes the extended relative jacobian given the current joint state and the virtual sticks to the task C-frame.
    **/
    Eigen::MatrixXd computeJacobian(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2) const;

    double alpha_, damping_;
  };
}

#endif
