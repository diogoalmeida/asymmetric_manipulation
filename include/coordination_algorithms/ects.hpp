#ifndef __ECTS__
#define __ECTS__

#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
/**
  Implements the extended cooperative task space (ECTS) algorithm
**/
class ECTS : public AlgorithmBase
{
 public:
  ECTS();
  ~ECTS() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

  void getAbsoluteVelocity(const sensor_msgs::JointState &state,
                           const Vector3d &r1, const Vector3d &r2,
                           Vector6d &abs_vel) const;
  void getRelativeVelocity(const sensor_msgs::JointState &state,
                           const Vector3d &r1, const Vector3d &r2,
                           Vector6d &rel_vel) const;
  KDL::Frame getAbsoluteMotionFrame(const KDL::Frame &obj1,
                                    const KDL::Frame &obj2) const;
  KDL::Frame getRelativeMotionFrame(const KDL::Frame &obj1,
                                    const KDL::Frame &obj2) const;

 private:
  /**
    Computes the ECTS jacobian given the current joint state and the virtual
  sticks to the task C-frame.
  **/
  Eigen::MatrixXd computeJacobian(const sensor_msgs::JointState &state,
                                  const Vector3d &r1, const Vector3d &r2) const;
};
}  // namespace coordination_algorithms

#endif
