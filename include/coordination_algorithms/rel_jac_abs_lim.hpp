#ifndef __RELATIVE_JACOBIAN_ABSOLUTE_LIMITS__
#define __RELATIVE_JACOBIAN_ABSOLUTE_LIMITS__

#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
const int LEFT = 0, RIGHT = 1;
/**
  Implements the extended Jacobian algorithm, with avoiding absolute limits as a
secundary task.
**/
class RelJacAbsLim : public AlgorithmBase
{
 public:
  RelJacAbsLim();
  ~RelJacAbsLim() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

  /**
    Gets the target pose for the secundary task by querying TF.

    @returns False if TF query fails.
  **/
  bool getSecundaryTask();

 private:
  /**
    Loads workspace limits to be used by the secundary task.
  **/
  bool init();

  /**
    Compute the absolute motion task command.

    @param abs_pose The absolute pose of the system.
  **/
  Vector6d computeAbsTask(const geometry_msgs::Pose &abs_pose) const;

  int max_tf_attempts_;
  tf::TransformListener listener_;
  std::map<int, KDL::Frame> obj_in_eef_;
  std::map<int, std::string> obj_frame_, eef_frame_;
  std::vector<double> pos_upper_ct_, pos_upper_thr_, pos_lower_ct_,
      pos_lower_thr_;
};
}  // namespace coordination_algorithms

#endif
