#!/usr/bin/env python
import sys

import actionlib
import rospy
import tf
from coordination_experiments.msg import (
    CoordinationControllerAction, CoordinationControllerFeedback,
    CoordinationControllerGoal, RunSimulationAction)
from generic_control_toolbox.manage_actionlib import monitor_action_goal
from geometry_msgs.msg import PoseStamped, TransformStamped
from object_server.srv import SetMarkers, SetMarkersRequest
from std_srvs.srv import Empty, SetBool

init_obj_frames = [{
    'left': [(0.5, 0, 0.2), (0, 0, 0.55774, 0.830016)],
    'right': [(0.5, 0, 0.2), (0, 0, 0, 1)]
}, {
    'left': [(0, 0, 0), (0, 0, 0, 1)],
    'right': [(0, 0, 0), (0, 0, 0, 1)]
}]

base_frame = 'torso'
frame_names = {'left': 'left_object', 'right': 'right_object'}
parent_frames = {'left': 'left_gripper', 'right': 'right_gripper'}


def setManipulationTargets(iter, list):
    """Sets the object frames according to the current iteration num.

       @param iter Current iteration
       @param list TF listener
    """
    if iter >= len(init_obj_frames):
        raise Exception("Iter larger than num of pre-set frames")

    obj_server = rospy.ServiceProxy('/object_server/set_marker_pose',
                                    SetMarkers)

    req = SetMarkersRequest()
    req.marker_name = []

    for arm in ['left', 'right']:
        pose = init_obj_frames[iter][arm]
        pmsg = PoseStamped()
        pmsg.header.frame_id = base_frame
        pmsg.pose.position.x = pose[0][0]
        pmsg.pose.position.y = pose[0][1]
        pmsg.pose.position.z = pose[0][2]
        pmsg.pose.orientation.x = pose[1][0]
        pmsg.pose.orientation.y = pose[1][1]
        pmsg.pose.orientation.z = pose[1][2]
        pmsg.pose.orientation.w = pose[1][3]
        msg_in_parent = list.transformPose(parent_frames[arm], pmsg)

        req.marker_name += [frame_names[arm]]
        req.marker_pose += [msg_in_parent.pose]

    resp = obj_server(req)


if __name__ == "__main__":
    rospy.init_node("run_sims")
    top_server = actionlib.SimpleActionServer(
        "/coordination_simulations/initialize", RunSimulationAction)
    coordination_action_name = "coordination_controller/coordination_control"
    coordination_client = actionlib.SimpleActionClient(
        coordination_action_name, CoordinationControllerAction)
    object_server_name = "/object_server/toggle_marker_update"
    object_client = rospy.ServiceProxy(object_server_name, SetBool)
    sim_reset = rospy.ServiceProxy("/state_reset", Empty)
    list = tf.TransformListener()

    rospy.loginfo("Waiting for coordination action server...")
    coordination_client.wait_for_server()

    while not rospy.is_shutdown():
        while not (top_server.is_new_goal_available() or rospy.is_shutdown()):
            rospy.loginfo_throttle(
                60, "Run simulations action server waiting for goal...")
            rospy.sleep(0.5)

        if rospy.is_shutdown():
            sys.exit()

        goal = top_server.accept_new_goal()
        coordination_goal = CoordinationControllerGoal()
        coordination_goal.max_time = 5.0
        coordination_goal.alpha = 0.5
        try:
            resp = object_client(False)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to contact the object server: %s" % e)
            top_server.set_aborted()
            continue

        try:
            resp = sim_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to contact the simulation: %s" % e)
            top_server.set_aborted()
            continue

        setManipulationTargets(0, list)

        # Run ECTS
        rospy.loginfo("Initializing ECTS simulation")
        coordination_goal.control_mode.controller = coordination_goal.control_mode.ECTS

        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        # run relative Jacobian (master-slave)
        rospy.loginfo("Initializing relative Jacobian ('master-slave')")
        coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJAC

        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        # run relative Jacobian (absolute limits)
        rospy.loginfo("Initializing relative Jacobian (absolute limits)")
        coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJACABSLIM
        coordination_goal.limits.position_max_limits.x = 0.6
        coordination_goal.limits.position_max_limits.y = 0.2
        coordination_goal.limits.position_max_limits.z = 0.3
        coordination_goal.limits.position_min_limits.x = 0.4
        coordination_goal.limits.position_min_limits.y = -0.2
        coordination_goal.limits.position_min_limits.z = -0.1
        coordination_goal.limits.threshold_distance = 0.1
        coordination_goal.limits.orientation_limit_angle = 10.5
        coordination_goal.limits.orientation_threshold = 0.5
        coordination_goal.symmetric_secundary_task = False

        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        rospy.loginfo(
            "Initializing relative Jacobian (symmetric absolute limits)")
        coordination_goal.symmetric_secundary_task = True

        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        # run extended relative Jacobian
        rospy.loginfo("Initializing extended relative Jacobian")
        coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
        coordination_goal.dynamic_alpha = True
        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        if success:
            top_server.set_succeeded()
