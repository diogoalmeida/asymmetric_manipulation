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

init_obj_frames = {'I': [{
    'left': [(0.405, 0, 0.2), (0, 0, 0.3704, 0.92887)],
    'right': [(0.405, 0, 0.2), (0, 0, -0.10454, 0.99452)]
}],
    'II': [{
        'left': [(0, 0, 0), (0, 0, 0, 1)],
        'right': [(0, 0, 0), (0, 0, 0, 1)]
    }],
    'III': [{
        'left': [(0.54125, -0.1, 0.16), (-0.018431, -0.09522, 0.34287, 0.93436)],
        'right': [(0.54125, -0.1, 0.16), (0.17814, 0.28162, 0.71979, 0.60898)]
    }, {
        'left': [(0.61275, -0.08, 0.24216), (-0.016279, -0.091959, 0.12224, 0.9881)],
        'right': [(0.61275, -0.08, 0.24216), (-0.076637, -0.048926, 0.78317, 0.61512)]
    }]}

base_frame = 'torso'
frame_names = {'left': 'left_object', 'right': 'right_object'}
parent_frames = {'left': 'left_gripper', 'right': 'right_gripper'}

init_time = rospy.Time(0)
t = np.array([])
abs_pose = np.array(None)
manip1 = np.array([])
manip2 = np.array([])
computed_alpha = np.array([])


def resetVars():
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, init_time
    init_time = rospy.Time(0)
    t = np.array([])
    abs_pose = np.array(None)
    manip1 = np.array([])
    manip2 = np.array([])
    computed_alpha = np.array([])


def feedbackCb(m):
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, init_time
    if init_time == rospy.Time(0):
        init_time = rospy.Time.now()

    t = np.append(t, [(rospy.Time.now() - init_time).to_sec()])
    if abs_pose.any() == None:
        abs_pose = np.array([[m.feedback.absolute_position.x,
                              m.feedback.absolute_position.y,
                              m.feedback.absolute_position.z]])
    else:
        abs_pose = np.append(abs_pose, [[m.feedback.absolute_position.x,
                                         m.feedback.absolute_position.y,
                                         m.feedback.absolute_position.z]], axis=0)
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


def setManipulationTargets(case, iter, list):
    """Sets the object frames according to the current iteration num.

       @param case Test case
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
        pose = init_obj_frames[case][iter][arm]
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
    """Should plot the absolute trajectory of a system run."""
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


def makeSecondCasePlot(color='k', lbl=None):
    """Plot the effective degree of sharing."""
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    matplotlib.rcParams['figure.figsize'] = (14, 9)
    fig = plt.figure(1)
    # plt.plot(t[1:], effective_alpha[1:], color=color, label=lbl)
    plt.xlabel('Time [s]')
    plt.ylabel('Effective ' + r'$\alpha$')


def makeThirdCasePlot(c_alpha='k', c_mu1='c', c_mu2='darkorange', line='-', lbl=None):
    """Plot the execution alpha and the manipulability indices."""
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    matplotlib.rcParams['figure.figsize'] = (14, 9)
    fig = plt.figure(1)
    plt.subplot(211)
    plt.plot(t[1:], computed_alpha[1:], color=c_alpha, label=lbl)
    plt.ylabel(r'$\alpha$')
    plt.subplot(212)
    plt.plot(t[1:], manip1[1:], color=c_mu1, linestyle=line, label=r"$\mu_1$")
    plt.plot(t[1:], manip2[1:], color=c_mu2, linestyle=line, label=r"$\mu_2$")
    plt.xlabel('Time [s]')
    plt.legend()


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

        success = False
        # TEST CASE I
        if goal.case_one:
            setManipulationTargets('I', 0, list)
            resetVars()
            # run relative Jacobian (master-slave)
            rospy.loginfo("Initializing relative Jacobian ('master-slave')")
            coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJAC

            success = monitor_action_goal(
                top_server,
                coordination_client,
                coordination_goal,
                action_name=coordination_action_name)

            if not success:
                top_server.set_aborted()
                continue

            makeFirstCasePlot('r', 'Relative Jacobian')

            coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
            coordination_goal.dynamic_alpha = False
            for alpha in range(0, 11, 1):
                resetVars()
                coordination_goal.alpha = alpha / 10.
                success = monitor_action_goal(
                    top_server,
                    coordination_client,
                    coordination_goal,
                    action_name=coordination_action_name)

                if not success:
                    top_server.set_aborted()
                    break

                makeFirstCasePlot(str(alpha / 10.),
                                  r'$\alpha = $' + ' ' + str(alpha))

            if not success:
                break

            plt.show()

            try:
                resp = sim_reset()
            except rospy.ServiceException, e:
                rospy.logerr("Failed to contact the simulation: %s" % e)
                top_server.set_aborted()
                continue

        # TEST CASE II
        if goal.case_two:
            resetVars()
            setManipulationTargets('II', 0, list)

            # run relative Jacobian (absolute limits)
            rospy.loginfo("Initializing relative Jacobian (absolute limits)")
            coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJACABSLIM
            coordination_goal.limits.position_max_limits.x = 0.3
            coordination_goal.limits.position_max_limits.y = 0.2
            coordination_goal.limits.position_max_limits.z = 0.3
            coordination_goal.limits.position_min_limits.x = 0.0
            coordination_goal.limits.position_min_limits.y = -0.2
            coordination_goal.limits.position_min_limits.z = -0.1
            coordination_goal.limits.orientation_limit_angle = 0.4
            coordination_goal.symmetric_secundary_task = False

            success = monitor_action_goal(
                top_server,
                coordination_client,
                coordination_goal,
                action_name=coordination_action_name)

            if not success:
                top_server.set_aborted()
                continue

            makeSecondCasePlot('k')

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
            resetVars()
            rospy.loginfo("Initializing extended relative Jacobian")
            coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
            coordination_goal.alpha = 0.9
            coordination_goal.dynamic_alpha = False
            success = monitor_action_goal(
                top_server,
                coordination_client,
                coordination_goal,
                action_name=coordination_action_name)

            if not success:
                top_server.set_aborted()
                continue

            makeSecondCasePlot('b')
            plt.show()

        # TEST CASE III
        if goal.case_three:
            for i in range(len(init_obj_frames['III'])):
                try:
                    resp = sim_reset()
                except rospy.ServiceException, e:
                    rospy.logerr("Failed to contact the simulation: %s" % e)
                    top_server.set_aborted()
                    continue

                resetVars()
                setManipulationTargets('III', i, list)
                coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJACABSLIM
                coordination_goal.limits.position_max_limits.x = 10.3
                coordination_goal.limits.position_max_limits.y = 10.2
                coordination_goal.limits.position_max_limits.z = 10.3
                coordination_goal.limits.position_min_limits.x = -10.0
                coordination_goal.limits.position_min_limits.y = -10.2
                coordination_goal.limits.position_min_limits.z = -10.1
                coordination_goal.limits.orientation_limit_angle = 10.5
                coordination_goal.symmetric_secundary_task = False

                coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                coordination_goal.alpha = 0.5
                coordination_goal.dynamic_alpha = False
                success = monitor_action_goal(
                    top_server,
                    coordination_client,
                    coordination_goal,
                    action_name=coordination_action_name)

                if not success:
                    top_server.set_aborted()
                    continue

                makeThirdCasePlot('k', 'c', 'darkorange')

                resetVars()
                coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                coordination_goal.alpha = 0.5
                coordination_goal.dynamic_alpha = True
                success = monitor_action_goal(
                    top_server,
                    coordination_client,
                    coordination_goal,
                    action_name=coordination_action_name)

                if not success:
                    top_server.set_aborted()
                    continue

                makeThirdCasePlot('b', 'c', 'darkorange', '--')
                plt.show()

        top_server.set_succeeded()
