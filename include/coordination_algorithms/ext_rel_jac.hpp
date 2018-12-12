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
};
}  // namespace coordination_algorithms

#endif
