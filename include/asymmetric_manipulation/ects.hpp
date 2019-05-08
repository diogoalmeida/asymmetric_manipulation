#ifndef __ECTS__
#define __ECTS__

#include <asymmetric_manipulation/algorithm_base.hpp>

namespace coordination_algorithms
{
/**
  Implements the extended cooperative task space (ECTS) algorithm
**/
class ECTS : public AlgorithmBase
{
 public:
  ECTS(const Vector3d &pos_upper_ct, const Vector3d &pos_lower_ct,
       double ori_ct);
  ~ECTS() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

 private:
  bool init();
  Eigen::Affine3d secundary_target_;
  Eigen::MatrixXd Kp_;
  tf::TransformListener listener_;
};
}  // namespace coordination_algorithms

#endif
