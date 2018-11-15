#ifndef __ECTS_CONTROLLER__
#define __ECTS_CONTROLLER__

#include <ros/ros.h>
#include <coordination_algorithms/ects.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <generic_control_toolbox/controller_template.hpp>
#include <generic_control_toolbox/controller_action_node.hpp>
#include <coordination_experiments/CoordinationControllerAction.h>
#include <std_srvs/Empty.h>

const int LEFT = 0, RIGHT = 1;
namespace coordination_experiments
{
  class CoordinationController : public generic_control_toolbox::ControllerTemplate<CoordinationControllerAction,
                                                                                    CoordinationControllerGoal,
                                                                                    CoordinationControllerFeedback,
                                                                                    CoordinationControllerResult>
  {
  public:
    CoordinationController(const std::string &action_name);
    ~CoordinationController() {}

  protected:
    sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
    bool parseGoal(boost::shared_ptr<const CoordinationControllerGoal> goal);
    void resetController();

  private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
    std::shared_ptr<coordination_algorithms::AlgorithmBase> alg_;
    std::map<int, KDL::Frame> obj_in_eef_;
    std::map<int, std::string> eef_frame_, obj_frame_;
    std::map<int, unsigned int> num_joints_;
    double max_joint_pos_error_;
    Eigen::VectorXd target_joint_positions_;
    Eigen::MatrixXd Kp_r_;
    bool newGoal_;
    int max_tf_attempts_;
    ros::ServiceClient reset_client_;

    /**
      Initialize experiment parameters. This will set up rigid transforms between the
      dual-armed system and the pre-defined object frames attached to each end-effector.

      @returns True if initialization is successful, false otherwise.
    **/
    bool init();
  };
}

#endif
