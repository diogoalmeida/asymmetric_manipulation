#include <coordination_algorithms/ext_rel_jac.hpp>

namespace coordination_algorithms
{
  ExtRelJac::ExtRelJac() : AlgorithmBase()
  {
    // TODO: Parameters
    alpha_ = 0.5;
    damping_ = 0.01;
  }

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

  Eigen::Matrix3d ExtRelJac::getRelativeToBase(const KDL::Frame &obj1, const KDL::Frame &obj2) const
  {
    KDL::Vector r1_rot_vec = obj1.M.GetRot();
    double r1_rot_angle = r1_rot_vec.Normalize();

    KDL::Rotation r1_interp = KDL::Rotation::Rot(r1_rot_vec, r1_rot_angle*(1-alpha_)/((1-alpha_)*(1-alpha_) + alpha_*alpha_));
    Eigen::Matrix3d ret;
    ret << r1_interp.data[0], r1_interp.data[1], r1_interp.data[2],
           r1_interp.data[3], r1_interp.data[4], r1_interp.data[5],
           r1_interp.data[6], r1_interp.data[7], r1_interp.data[8];

    return ret;
  }

  Eigen::MatrixXd ExtRelJac::computeJacobian(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2) const
  {
    Matrix12d W = Matrix12d::Identity();
    Eigen::Matrix<double, 6, 12> L;
    Eigen::MatrixXd J_r, J;
    KDL::Jacobian J1_kdl, J2_kdl;

    kdl_manager_->getJacobian(eef1_, state, J1_kdl);
    kdl_manager_->getJacobian(eef2_, state, J2_kdl);

    L.block<6,6>(0,0) = -(1-alpha_)*Matrix6d::Identity();
    L.block<6,6>(0,6) = alpha_*Matrix6d::Identity();

    L = 1/((1-alpha_)*(1-alpha_) + alpha_*alpha_) * L;

    W.block<3,3>(0,3) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r1);
    W.block<3,3>(6,9) = -generic_control_toolbox::MatrixParser::computeSkewSymmetric(r2);

    J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
    J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
    J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

    J_r = L*W*J;

    return J_r;
  }
}
