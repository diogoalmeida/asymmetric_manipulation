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
  ExtRelJac(const std::vector<double> &pos_upper_ct,
            const std::vector<double> &pos_upper_thr,
            const std::vector<double> &pos_lower_ct,
            const std::vector<double> &pos_lower_thr,
            const std::vector<double> &ori_ct,
            const std::vector<double> &ori_thr);
  ~ExtRelJac() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

 private:
  /**
    Compute the appropriate value of alpha to constrain the absolute motion,
  such that the absolute workspace limits are respected.
  **/
  double computeAlpha(const Eigen::MatrixXd &J1,
                      const Eigen::MatrixXd &J2) const;
};
}  // namespace coordination_algorithms

#endif
