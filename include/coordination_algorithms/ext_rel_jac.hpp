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

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

 private:
  /**
   Compute alpha such that the current absolute pose (in the base frame) remains
   within user-specified bounds.

   @param abs_pose The current absolute position of the system.
   @param Ji Manipulators' Jacobians.
   @returns The new value for alpha, setting how the manipulators will cooperate
  in the relative motion task.
  **/
  double computeAlpha(const geometry_msgs::Pose &abs_pose,
                      const Eigen::MatrixXd &J1,
                      const Eigen::MatrixXd &J2) const;
};
}  // namespace coordination_algorithms

#endif
