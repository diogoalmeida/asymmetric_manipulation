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

    Matrix12d W = computeW(r1, r2);
    Eigen::Matrix<double, 6, 12> L = Eigen::Matrix<double, 6, 12>::Zero(), L_sim = Eigen::Matrix<double, 6, 12>::Zero();
    Eigen::MatrixXd J_a, J_o, J_sim;
    KDL::Jacobian J1_kdl, J2_kdl;

    kdl_manager_->getJacobian(eef1_, state, J1_kdl);
    kdl_manager_->getJacobian(eef2_, state, J2_kdl);

    L.block<6,6>(0,0) = (rel_alpha_)*Matrix6d::Identity();
    L.block<6,6>(0,6) = (1 - rel_alpha_)*Matrix6d::Identity();
    L_sim.block<6,6>(0,0) = -Matrix6d::Identity();
    L_sim.block<6,6>(0,6) = Matrix6d::Identity();

    J_o = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
    J_o.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
    J_o.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

    J_a = L*W*J_o;
    J_sim = L_sim*W*J_o;

    Eigen::Matrix<double, 14, 6> damped_a_inverse = J_a.transpose()*(J_a*J_a.transpose() + 0*damping_*Matrix6d::Identity()).inverse(), damped_sim_inverse = J_sim.transpose()*(J_sim*J_sim.transpose()).inverse();

    damped_inverse = (J*J.transpose() + 0*damping_*Matrix6d::Identity());

    q_dot = J.transpose()*damped_inverse.colPivHouseholderQr().solve(rel_twist);

    return damped_sim_inverse*rel_twist + (Eigen::Matrix<double, 14, 14>::Identity() - damped_sim_inverse*J_sim)*q_dot;
  }

  void ExtRelJac::getAbsoluteVelocity(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2, Vector6d &abs_vel) const
  {
    KDL::Twist v1_eef, v2_eef;
    Vector6d v1_eef_eig, v2_eef_eig, v1, v2;
    Vector12d total_twist;
    Eigen::Matrix<double, 6, 12> absL = Eigen::Matrix<double, 6, 12>::Zero();
    Matrix12d W = computeW(r1, r2);
    kdl_manager_->getGrippingTwist(eef1_, state, v1_eef);
    kdl_manager_->getGrippingTwist(eef2_, state, v2_eef);

    tf::twistKDLToEigen(v1_eef, v1_eef_eig);
    tf::twistKDLToEigen(v2_eef, v2_eef_eig);

    total_twist.block<6,1>(0, 0) = v1_eef_eig;
    total_twist.block<6,1>(6, 0) = v2_eef_eig;

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
    Matrix12d W = computeW(r1, r2);
    kdl_manager_->getGrippingTwist(eef1_, state, v1_eef);
    kdl_manager_->getGrippingTwist(eef2_, state, v2_eef);

    tf::twistKDLToEigen(v1_eef, v1_eef_eig);
    tf::twistKDLToEigen(v2_eef, v2_eef_eig);

    total_twist.block<6,1>(0, 0) = v1_eef_eig;
    total_twist.block<6,1>(6, 0) = v2_eef_eig;

    relL.block<6,6>(0, 0) = -(1 - rel_alpha_)*Matrix6d::Identity();
    relL.block<6,6>(0, 6) = rel_alpha_*Matrix6d::Identity();

    relL = 1/((1-rel_alpha_)*(1-rel_alpha_) + rel_alpha_*rel_alpha_) * relL;

    rel_vel = relL*W*total_twist;
  }

  KDL::Frame ExtRelJac::getAbsoluteMotionFrame(const KDL::Frame &obj1, const KDL::Frame &obj2) const
  {
    Eigen::Quaterniond obj1_orientation, relative_orientation;

    tf::quaternionKDLToEigen(obj1.M.Inverse()*obj2.M, relative_orientation);
    tf::quaternionKDLToEigen(obj1.M, obj1_orientation);

    Eigen::AngleAxisd relative_angle_axis(relative_orientation);
    Eigen::AngleAxisd absolute_angle_axis((1 - abs_alpha_)*relative_angle_axis.angle(), relative_angle_axis.axis());

    Eigen::Quaterniond absolute_orientation(obj1_orientation*absolute_angle_axis);

    KDL::Frame absolute_frame;

    tf::quaternionEigenToKDL(absolute_orientation, absolute_frame.M);

    absolute_frame.p = abs_alpha_*obj1.p + (1 - abs_alpha_)*obj2.p;

    return absolute_frame;
  }

  KDL::Frame ExtRelJac::getRelativeMotionFrame(const KDL::Frame &obj1, const KDL::Frame &obj2) const
  {
    Eigen::Quaterniond obj1_rot, obj2_rot;
    KDL::Frame relative_frame;

    tf::quaternionKDLToEigen(obj1.M, obj1_rot);
    tf::quaternionKDLToEigen(obj2.M, obj2_rot);

    Eigen::AngleAxisd obj1_ang_axis(obj1_rot), obj2_ang_axis(obj2_rot);
    double scaling = 1/((1 - rel_alpha_)*(1 - rel_alpha_) + rel_alpha_*rel_alpha_);
    Eigen::AngleAxisd new_obj1_ang_axis((1 - rel_alpha_)*scaling*obj1_ang_axis.angle(), obj1_ang_axis.axis()), new_obj2_ang_axis(rel_alpha_*scaling*obj2_ang_axis.angle(), obj2_ang_axis.axis());

    Eigen::Quaterniond relative_orientation(new_obj2_ang_axis), new_obj1_orientation(new_obj1_ang_axis);
    KDL::Rotation new_obj1_rot;

    tf::quaternionEigenToKDL(relative_orientation, relative_frame.M);
    tf::quaternionEigenToKDL(new_obj1_orientation, new_obj1_rot);
    relative_frame.p = new_obj1_rot*(scaling*(rel_alpha_*obj2.p - (1 - rel_alpha_)*obj1.p));

    return relative_frame;
  }

  Eigen::MatrixXd ExtRelJac::computeJacobian(const sensor_msgs::JointState &state, const Vector3d &r1, const Vector3d &r2) const
  {
    Matrix12d W = computeW(r1, r2);
    double scaling = 1/((1-rel_alpha_)*(1-rel_alpha_) + rel_alpha_*rel_alpha_);
    Eigen::Matrix<double, 6, 12> L = Eigen::Matrix<double, 6, 12>::Zero();
    Eigen::MatrixXd J_r, J;
    KDL::Jacobian J1_kdl, J2_kdl;

    kdl_manager_->getJacobian(eef1_, state, J1_kdl);
    kdl_manager_->getJacobian(eef2_, state, J2_kdl);

    L.block<6,6>(0,0) = -(1 - rel_alpha_)*scaling*Matrix6d::Identity();
    // L.block<6,6>(0,0) = -Matrix6d::Identity();
    L.block<6,6>(0,6) = rel_alpha_*scaling*Matrix6d::Identity();
    // L.block<6,6>(0,6) = Matrix6d::Identity();

    J = Eigen::MatrixXd::Zero(12, J1_kdl.columns() + J2_kdl.columns());
    J.block(0, 0, 6, J1_kdl.columns()) = J1_kdl.data;
    J.block(6, J1_kdl.columns(), 6, J2_kdl.columns()) = J2_kdl.data;

    J_r = L*W*J;

    return J_r;
  }
}
