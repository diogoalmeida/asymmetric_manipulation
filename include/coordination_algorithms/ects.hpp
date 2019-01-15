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
  ECTS(const std::vector<double> &pos_upper_ct,
       const std::vector<double> &pos_upper_thr,
       const std::vector<double> &pos_lower_ct,
       const std::vector<double> &pos_lower_thr,
       const std::vector<double> &ori_ct, const std::vector<double> &ori_thr);
  ~ECTS() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);
};
}  // namespace coordination_algorithms

#endif
