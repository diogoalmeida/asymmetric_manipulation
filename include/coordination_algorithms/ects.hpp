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

    Eigen::VectorXd control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &abs_twist, const Vector6d &rel_twist);

  private:
    /**
      Computes the ECTS jacobian given the current joint state and the virtual sticks to the task C-frame.
    **/
    Eigen::MatrixXd computeJacobian(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2) const;

    double alpha_, damping_;
  };
}

#endif
