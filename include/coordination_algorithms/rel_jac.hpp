#ifndef __RELATIVE_JACOBIAN__
#define __RELATIVE_JACOBIAN__

#include <coordination_algorithms/algorithm_base.hpp>

namespace coordination_algorithms
{
/**
  Implements the extended Jacobian algorithm, with a secundary task for one of
the end-effectors, based on the literature which uses the relative Jacobian to
coordinate a dual-armed system.
**/
class RelJac : public AlgorithmBase
{
 public:
  RelJac();
  ~RelJac() {}

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
   Sets up an interactive marker to acquire the secundary task target and gains.

   @returns False case parameters are missing for the secundary task.
  **/
  bool init();

  Eigen::Affine3d secundaryTarget_;
};
}  // namespace coordination_algorithms

#endif
