#include <coordination_algorithms/ext_rel_jac.hpp>

namespace coordination_algorithms
{
  ExtRelJac::ExtRelJac() : AlgorithmBase() {}

  Eigen::VectorXd ExtRelJac::control(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, const Vector6d &abs_twist, const Vector6d &rel_twist)
  {
    Eigen::MatrixXd J;
    Matrix6d damped_inverse;
    Eigen::VectorXd q_dot;
    KDL::Frame p1, p2, eef1, eef2;
    Eigen::Affine3d p1_eig, p2_eig, eef1_eig, eef2_eig;

    kdl_manager_->getGrippingPoint(eef1_, state, p1);
    kdl_manager_->getGrippingPoint(eef2_, state, p2);
    kdl_manager_->getEefPose(eef1_, state, eef1);
    kdl_manager_->getEefPose(eef2_, state, eef2);

    tf::transformKDLToEigen(p1, p1_eig);
    tf::transformKDLToEigen(p2, p2_eig);
    tf::transformKDLToEigen(eef1, eef1_eig);
    tf::transformKDLToEigen(eef2, eef2_eig);

    J = computeJacobian(state, r1 + p1_eig.translation() - eef1_eig.translation(), r2 + p2_eig.translation() - eef2_eig.translation());

    damped_inverse = (J*J.transpose() + damping_*Matrix12d::Identity());

    q_dot = J.transpose()*damped_inverse.colPivHouseholderQr().solve(rel_twist);

    return q_dot;
  }

  void ExtRelJac::getAbsoluteVelocity(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, Vector6d &abs_vel) const
  {
    KDL::Twist v1_eef, v2_eef;
    Vector6d v1_eef_eig, v2_eef_eig, v1, v2;
    Vector12d total_twist;
    Eigen::Matrix<double, 6, 12> absL = Eigen::Matrix<double, 6, 12>::Zero();
    Matrix12d W = Matrix12d::Identity();
    kdl_manager_->getGrippingTwist(eef1_, state, v1_eef);
    kdl_manager_->getGrippingTwist(eef2_, state, v2_eef);

    tf::twistKDLToEigen(v1_eef, v1_eef_eig);
    tf::twistKDLToEigen(v2_eef, v2_eef_eig);

    total_twist.block<6,1>(0, 0) = v1_eef_eig;
    total_twist.block<6,1>(6, 0) = v2_eef_eig;

    W.block<3,3>(0,3) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r1);
    W.block<3,3>(6,9) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r2);

    absL.block<6,6>(0, 0) = abs_alpha_*Matrix6d::Identity();
    absL.block<6,6>(0, 6) = (1 - abs_alpha_)*Matrix6d::Identity();

    abs_vel = absL*W*total_twist;
  }

  void ExtRelJac::getRelativeVelocity(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, Vector6d &rel_vel) const
  {
    KDL::Twist v1_eef, v2_eef;
    Vector6d v1_eef_eig, v2_eef_eig, v1, v2;
    Vector12d total_twist;
    Eigen::Matrix<double, 6, 12> relL = Eigen::Matrix<double, 6, 12>::Zero();
    Matrix12d W = Matrix12d::Identity();
    kdl_manager_->getGrippingTwist(eef1_, state, v1_eef);
    kdl_manager_->getGrippingTwist(eef2_, state, v2_eef);

    tf::twistKDLToEigen(v1_eef, v1_eef_eig);
    tf::twistKDLToEigen(v2_eef, v2_eef_eig);

    total_twist.block<6,1>(0, 0) = v1_eef_eig;
    total_twist.block<6,1>(6, 0) = v2_eef_eig;

    W.block<3,3>(0,3) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r1);
    W.block<3,3>(6,9) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r2);

    relL.block<6,6>(0, 0) = -(1 - rel_alpha_)*Matrix6d::Identity();
    relL.block<6,6>(0, 6) = rel_alpha_*Matrix6d::Identity();

    relL = 1/((1-rel_alpha_)*(1-rel_alpha_) + rel_alpha_*rel_alpha_) * relL;

    rel_vel = relL*W*total_twist;
  }

  Eigen::MatrixXd ExtRelJac::computeJacobian(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2) const
  {
    Matrix12d W = Matrix12d::Identity();
    Eigen::Matrix<double, 6, 12> L;
    Eigen::MatrixXd J_r, J;
    KDL::Jacobian J1_kdl, J2_kdl;

    kdl_manager_->getJacobian(eef1_, state, J1_kdl);
    kdl_manager_->getJacobian(eef2_, state, J2_kdl);

    L.block<6,6>(0,0) = -(1 - rel_alpha_)*Matrix6d::Identity();
    L.block<6,6>(0,6) = rel_alpha_*Matrix6d::Identity();

    L = 1/((1-rel_alpha_)*(1-rel_alpha_) + rel_alpha_*rel_alpha_) * L;

    W.block<3,3>(0,3) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r1);
    W.block<3,3>(6,9) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r2);

    J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
    J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
    J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

    J_r = L*W*J;

    return J_r;
  }
}
