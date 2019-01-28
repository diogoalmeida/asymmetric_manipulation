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
  ExtRelJac(const Vector3d &pos_upper_ct, const Vector3d &pos_lower_ct,
            double ori_ct, bool onlyl);
  ~ExtRelJac() {}

  Eigen::VectorXd control(const sensor_msgs::JointState &state,
                          const Vector3d &r1, const Vector3d &r2,
                          const Vector6d &abs_twist, const Vector6d &rel_twist);

 private:
  bool onlyl_;
  /**
    Initializes the controller parameters.
  **/
  bool init();

  /**
    Computes the value of alpha to allocate more of the relative task to the
    manipulator with higher manipulability.
  **/
  double computeAlpha(const Eigen::MatrixXd &J1,
                      const Eigen::MatrixXd &J2) const;

  /**
    Compute the asymmetric relatice Jacobian.

    @param J The diagonal of the two arms' Jacobians.
    @param W The wrench conversion matrix.
    @param alpha The desired degree of cooperation.
    @returns The asymmetric relative Jacobian.
  **/
  Eigen::MatrixXd computeAsymJac(const Eigen::MatrixXd &J, const Matrix12d &W,
                                 double alpha) const;
};
}  // namespace coordination_algorithms

#endif
