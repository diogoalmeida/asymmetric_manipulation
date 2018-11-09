#ifndef __ECTS__
#define __ECTS__

#include <coordination_algorithms/experiment_base.hpp>

namespace coordination_algorithms
{
  /**
    Implements the extended cooperative task space (ECTS) algorithm
  **/
  class ECTS : public ExperimentBase
  {
  public:
    ECTS();
    ~ECTS() {}

    Eigen::VectorXd control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &abs_twist, const Vector6d &rel_twist);
  };
}

#endif
