#ifndef __RELATIVE_JACOBIAN_ABSOLUTE_LIMITS__
#define __RELATIVE_JACOBIAN_ABSOLUTE_LIMITS__

#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
/**
  Implements the extended Jacobian algorithm, with avoiding absolute limits as a
secundary task.
**/
class RelJacAbsLim : public AlgorithmBase
{
 public:
  RelJacAbsLim(const std::vector<double> &pos_upper_ct,
               const std::vector<double> &pos_upper_thr,
               const std::vector<double> &pos_lower_ct,
               const std::vector<double> &pos_lower_thr,
               const std::vector<double> &ori_ct,
               const std::vector<double> &ori_thr);
  ~RelJacAbsLim() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

 private:
  bool init();

  /**
    Compute the absolute motion task command.

    @param abs_pose The absolute pose of the system.
  **/
  Vector6d computeAbsTask(const geometry_msgs::Pose &abs_pose) const;
};
}  // namespace coordination_algorithms

#endif
