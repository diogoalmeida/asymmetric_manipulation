#ifndef __COORDINATION_CONTROLLER__
#define __COORDINATION_CONTROLLER__

#include <asymmetric_manipulation/CoordinationControllerAction.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <asymmetric_manipulation/ects.hpp>
#include <asymmetric_manipulation/ext_rel_jac.hpp>
#include <asymmetric_manipulation/rel_jac.hpp>
#include <asymmetric_manipulation/rel_jac_abs_lim.hpp>
#include <generic_control_toolbox/controller_action_node.hpp>
#include <generic_control_toolbox/controller_template.hpp>
#include <mutex>
#include <robot_kinematic_simulation/embedded_sim.hpp>

const int LEFT = 0, RIGHT = 1;
enum ControlType
{
  align,
  twist
};
namespace coordination_experiments
{
class CoordinationController
    : public generic_control_toolbox::ControllerTemplate<
          asymmetric_manipulation::CoordinationControllerAction,
          asymmetric_manipulation::CoordinationControllerGoal,
          asymmetric_manipulation::CoordinationControllerFeedback,
          asymmetric_manipulation::CoordinationControllerResult>
{
 public:
  CoordinationController(const std::string &action_name);
  ~CoordinationController() {}

 protected:
  sensor_msgs::JointState controlAlgorithm(
      const sensor_msgs::JointState &current_state, const ros::Duration &dt);
  bool parseGoal(boost::shared_ptr<
                 const asymmetric_manipulation::CoordinationControllerGoal>
                     goal);
  void resetController();

 private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
  std::shared_ptr<coordination_algorithms::AlgorithmBase> alg_;
  std::map<int, KDL::Frame> obj_in_eef_;
  std::map<int, std::string> eef_frame_, obj_frame_;
  std::map<int, unsigned int> num_joints_;
  double max_joint_pos_error_, max_time_;
  Eigen::VectorXd target_joint_positions_;
  Eigen::MatrixXd Kp_r_, Kp_a_;
  Eigen::VectorXd q1_init_, q2_init_;
  Eigen::Matrix<double, 6, 1> commanded_rel_twist_;
  bool newGoal_;
  int max_tf_attempts_;
  ControlType control_type_;
  ros::ServiceClient reset_client_;
  ros::Subscriber twist_sub_;
  ros::Time init_time_;
  Eigen::Vector3d pos_upper_ct_, pos_lower_ct_;
  KDL::Frame absolute_target_;
  double ori_ct_;
  rviz_visual_tools::RvizVisualToolsPtr pos_ws_pub_;
  rviz_visual_tools::RvizVisualToolsPtr ori_ws_pub_;
  std::string base_;
  std::mutex alg_mtx_;

  /**
    Initialize experiment parameters. This will set up rigid transforms between
  the dual-armed system and the pre-defined object frames attached to each
  end-effector.

    @returns True if initialization is successful, false otherwise.
  **/
  bool init();

  /**
    Initialize the object frames to be aligned in ALIGN_FRAMES input mode.
    @return false if frames can't be found.
  **/
  bool initializeObjectFrames();

  void twistCommandCb(const geometry_msgs::TwistStamped::ConstPtr &msg);

  /**
    Publishes updates the frame where orientation limits are displayed.

    @param state Current joint state.
  **/
  void updateOrientationTransform(const sensor_msgs::JointState &state);

  /**
    Compute the relative motion twist for an align task.
  **/
  Eigen::Matrix<double, 6, 1> computeAlignRelativeTwist(
      const sensor_msgs::JointState &state);

  /**
    Compute the absolute motion twist.
  **/
  Eigen::Matrix<double, 6, 1> computeAlignAbsoluteTwist(
      const sensor_msgs::JointState &state);

  /**
    Compute the (symmetric) absolute pose of the two manipulators.

    @param state The current dual-arm system joint state.
  **/
  geometry_msgs::Pose computeAbsolutePose(const sensor_msgs::JointState &state);

  /**
    Compute alpha such that the current absolute pose (in the base frame)
    remains within user-specified bounds.

    @param abs_pose The current absolute position of the system.
    @param Ji Manipulators' Jacobians.
    @returns The new value for alpha, setting how the manipulators will
    cooperate in the relative motion task.
  **/
  double computeAlpha(const geometry_msgs::Pose &abs_pose,
                      const Eigen::MatrixXd &J1,
                      const Eigen::MatrixXd &J2) const;

  /**
    Compute the virtual sticks for an align task.
  **/
  void computeAlignVirtualSticks(const sensor_msgs::JointState &state,
                                 Eigen::Vector3d &r1, Eigen::Vector3d &r2);

  /**
    Compute the virtual sticks connecting the end-effectors to the C-Frame where
  the commanded twist is expressed.

    Assumption: the object frame rigidly attached to eef1 is the C-Frame
  **/
  void computeTwistVirtualSticks(const sensor_msgs::JointState &state,
                                 Eigen::Vector3d &r1, Eigen::Vector3d &r2);
};
}  // namespace coordination_experiments

#endif
