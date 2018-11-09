#include <coordination_algorithms/ects.hpp>

namespace coordination_algorithms
{
  ECTS::ECTS() : ExperimentBase() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &abs_twist, const Vector6d &rel_twist) 
  {
    return Eigen::Matrix<double, 14, 1>::Zero();
  }
}
