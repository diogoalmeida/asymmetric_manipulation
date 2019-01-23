#!/usr/bin/env python
import sys

import actionlib
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np
import rospy
import tf
from coordination_experiments.msg import (CoordinationControllerAction,
                                          CoordinationControllerFeedback,
                                          CoordinationControllerGoal,
                                          RunSimulationAction)
from generic_control_toolbox.manage_actionlib import monitor_action_goal
from geometry_msgs.msg import PoseStamped, TransformStamped
from mpl_toolkits.mplot3d import Axes3D
from object_server.srv import SetMarkers, SetMarkersRequest
from std_srvs.srv import Empty, SetBool

init_obj_frames = [{
    'left': [(0.205, 0, 0.2), (0, 0, 0.3704, 0.92887)],
    'right': [(0.205, 0, 0.2), (0, 0, -0.10454, 0.99452)]
}, {
    'left': [(0, 0, 0), (0, 0, 0, 1)],
    'right': [(0, 0, 0), (0, 0, 0, 1)]
}]

base_frame = 'torso'
frame_names = {'left': 'left_object', 'right': 'right_object'}
parent_frames = {'left': 'left_gripper', 'right': 'right_gripper'}

init_time = rospy.Time(0)
t = np.array([])
abs_pose = np.array(None)
effective_alpha = np.array([])
manip1 = np.array([])
manip2 = np.array([])
computed_alpha = np.array([])


def resetVars():
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    init_time = rospy.Time(0)
    t = np.array([])
    abs_pose = np.array(None)
    effective_alpha = np.array([])
    manip1 = np.array([])
    manip2 = np.array([])
    computed_alpha = np.array([])


def feedbackCb(m):
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    t = np.append(t, [(rospy.Time.now() - init_time).to_sec()])
    if abs_pose.any() == None:
        abs_pose = np.array([[m.feedback.absolute_position.x,
                              m.feedback.absolute_position.y,
                              m.feedback.absolute_position.z]])
    else:
        abs_pose = np.append(abs_pose, [[m.feedback.absolute_position.x,
                                         m.feedback.absolute_position.y,
                                         m.feedback.absolute_position.z]], axis=0)
    effective_alpha = np.append(effective_alpha, [m.feedback.effective_alpha])
    manip1 = np.append(manip1, [m.feedback.manip1])
    manip2 = np.append(manip2, [m.feedback.manip2])
    computed_alpha = np.append(computed_alpha, [m.feedback.curr_alpha])


def saveFig(name):
    """Save the fig in the bag directory."""
    path = getDir() + "/" + name + ".svg"
    plt.savefig(path)


def addLabelledPlot(x, y, label, color='k'):
    """Add a plot with the given label."""
    handle, = plt.plot(x, y, color, label=label)
    axes = plt.gca()
    autoscaleBasedOn(axes, [handle])
    # plt.legend([handle], [label])


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


def makeFirstCasePlot(color='k', lbl=None):
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    matplotlib.rcParams['figure.figsize'] = (14, 9)
    fig = plt.figure(1)
    ax = fig.gca(projection='3d')

    ax.scatter(abs_pose[1, 0], abs_pose[1, 1],
               abs_pose[1, 2], edgecolors='k', s=100, marker='o', facecolors='w')
    ax.plot(abs_pose[1:, 0], abs_pose[1:, 1],
            abs_pose[1:, 2], color=color, label=lbl)
    ax.scatter(abs_pose[-1, 0], abs_pose[-1, 1],
               abs_pose[-1, 2], c='k', s=100, marker='x')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()


if __name__ == "__main__":
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
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
    rospy.Subscriber(coordination_action_name + "/feedback",
                     CoordinationControllerFeedback, feedbackCb)

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
        coordination_goal.limits.position_max_limits.x = 10.6
        coordination_goal.limits.position_max_limits.y = 10.2
        coordination_goal.limits.position_max_limits.z = 10.3
        coordination_goal.limits.position_min_limits.x = -10.4
        coordination_goal.limits.position_min_limits.y = -10.2
        coordination_goal.limits.position_min_limits.z = -10.1
        coordination_goal.limits.orientation_limit_angle = 10.5

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

        # TEST CASE I
        resetVars()
        # run relative Jacobian (master-slave)
        # rospy.loginfo("Initializing relative Jacobian ('master-slave')")
        # coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJAC
        #
        # success = monitor_action_goal(
        #     top_server,
        #     coordination_client,
        #     coordination_goal,
        #     action_name=coordination_action_name)
        #
        # if not success:
        #     top_server.set_aborted()
        #     continue
        #
        # makeFirstCasePlot('r', 'Relative Jacobian')
        #
        # coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
        # coordination_goal.dynamic_alpha = False
        # for alpha in range(0, 11, 1):
        #     resetVars()
        #     coordination_goal.alpha = alpha / 10.
        #     success = monitor_action_goal(
        #         top_server,
        #         coordination_client,
        #         coordination_goal,
        #         action_name=coordination_action_name)
        #
        #     if not success:
        #         top_server.set_aborted()
        #         break
        #
        #     makeFirstCasePlot(str(alpha / 10.),
        #                       r'$\alpha = $' + ' ' + str(alpha))
        #
        # if not success:
        #     break
        #
        # plt.show()

        # TEST CASE II
        setManipulationTargets(0, list)

        # run relative Jacobian (absolute limits)
        rospy.loginfo("Initializing relative Jacobian (absolute limits)")
        coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJACABSLIM
        coordination_goal.limits.position_max_limits.x = 0.3
        coordination_goal.limits.position_max_limits.y = 0.2
        coordination_goal.limits.position_max_limits.z = 0.3
        coordination_goal.limits.position_min_limits.x = 0.0
        coordination_goal.limits.position_min_limits.y = -0.2
        coordination_goal.limits.position_min_limits.z = -0.1
        coordination_goal.limits.threshold_distance = 0.1
        coordination_goal.limits.orientation_limit_angle = 0.5
        coordination_goal.limits.orientation_threshold = 0.5
        coordination_goal.symmetric_secundary_task = False

        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        if not success:
            top_server.set_aborted()
            continue

        # rospy.loginfo(
        #     "Initializing relative Jacobian (symmetric absolute limits)")
        # coordination_goal.symmetric_secundary_task = True
        #
        # success = monitor_action_goal(
        #     top_server,
        #     coordination_client,
        #     coordination_goal,
        #     action_name=coordination_action_name)

        # run extended relative Jacobian
        rospy.loginfo("Initializing extended relative Jacobian")
        coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
        coordination_goal.alpha = 0.9
        coordination_goal.dynamic_alpha = False
        success = monitor_action_goal(
            top_server,
            coordination_client,
            coordination_goal,
            action_name=coordination_action_name)

        if success:
            top_server.set_succeeded()
