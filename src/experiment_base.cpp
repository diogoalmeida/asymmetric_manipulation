#include <coordination_algorithms/experiment_base.hpp>

namespace coordination_algorithms
{
  ExperimentBase::ExperimentBase()
  {
    nh_ = ros::NodeHandle("~");

    if (!init())
    {
      throw std::logic_error("Missing parameters for the folding controller");
    }
  }

  bool ExperimentBase::init()
  {
    if (!nh_.getParam("kinematic_chain_base_link", base_))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_);

    if (!setArm("eef1", eef1_))
    {
      return false;
    }

    if (!setArm("eef2", eef2_))
    {
      return false;
    }

    return true;
  }

  bool ExperimentBase::setArm(const std::string &arm_name, std::string &eef_name)
  {
    generic_control_toolbox::ArmInfo info;

    if (!generic_control_toolbox::getArmInfo(arm_name, info))
    {
      return false;
    }

    eef_name = info.kdl_eef_frame;

    if (!generic_control_toolbox::setKDLManager(info, kdl_manager_))
    {
      return false;
    }

    return true;
  }
}
