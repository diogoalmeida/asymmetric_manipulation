#!/usr/bin/env python
import datetime
import os
import sys

import actionlib
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np
import rospkg
import rospy
import tf
from coordination_experiments.msg import (CoordinationControllerAction,
                                          CoordinationControllerFeedback,
                                          CoordinationControllerGoal,
                                          RunSimulationAction,
                                          RunSimulationFeedback)
from generic_control_toolbox.manage_actionlib import monitor_action_goal
from geometry_msgs.msg import PoseStamped, TransformStamped
from mpl_toolkits.mplot3d import Axes3D
from object_server.srv import SetMarkers, SetMarkersRequest
from std_srvs.srv import Empty, SetBool

init_obj_frames = {
    'I': [{
        'left': [(0.36, 0.15, 0.36), (0, 0, 0, 1)],
        'right': [(0.508, -0.13, -0.04), (0, 0, 0, 1)]
    }, {
        'left': [(0.445, -0.05, 0.21), (-0.024939, -0.011954, -0.053292, 0.9982)],
        'right': [(0.445, -0.05, 0.21), (-0.023947, -0.0081608, -0.46361, 0.88568)]
    }],
    'II': [{
        'left': [(0.36, 0.15, 0.36), (0, 0, 0, 1)],
        'right': [(0.508, -0.13, -0.04), (0, 0, 0, 1)]
    }, {
        'left': [(0.445, -0.05, 0.21), (-0.024939, -0.011954, -0.053292, 0.9982)],
        'right': [(0.445, -0.05, 0.21), (-0.023947, -0.0081608, -0.46361, 0.88568)]
    }],
    # 'III': [{
    #     'left': [(0.445, -0.05, 0.21), (-0.024939, -0.011954, -0.053292, 0.9982)],
    #     'right': [(0.445, -0.05, 0.21), (-0.023947, -0.0081608, -0.46361, 0.88568)]
    # }, {
    #     'left': [(0.445, -0.05, 0.21), (-0.023947, -0.0081608, -0.46361, 0.88568)],
    #     'right': [(0.445, -0.05, 0.21), (-0.024939, -0.011954, -0.053292, 0.9982)]
    # }, {
    #     'left': [(0.36, 0.15, 0.36), (0, 0, 0, 1)],
    #     'right': [(0.508, -0.13, -0.04), (0, 0, 0, 1)]
    # }, {
    #     'left': [(0.508, -0.13, -0.04), (0, 0, 0, 1)],
    #     'right': [(0.36, 0.15, 0.36), (0, 0, 0, 1)]
    # }]}
    'III': [{
        'left': [(0.54125, -0.1, 0.16), (-0.018431, -0.09522, 0.34287, 0.93436)],
        'right': [(0.54125, -0.1, 0.16), (0.17814, 0.28162, 0.71979, 0.60898)]
    }, {
        'left': [(0.61275, -0.08, 0.24216), (-0.016279, -0.091959, 0.12224, 0.9881)],
        'right': [(0.61275, -0.08, 0.24216), (-0.076637, -0.048926, 0.78317, 0.61512)]
    }, {
        'left': [(0.57, 0.21, 0.2), (0, 0, 0, 1)],
        'right': [(0.57, 0.21, 0.64), (0, 0, 0, 1)]
    }, {
        'left': [(0.45, 0.24, 0.51), (0, 0, 0.26211, 0.96504)],
        'right': [(0.6, -0.14, 0.64), (0, 0, 0.26211, 0.96504)]
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
relative_error = np.array(None)
relative_error_angle = np.array([])
va = np.array(None)
qnorm = 0


def resetVars():
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, init_time, relative_error, relative_error_angle, va, qnorm
    init_time = rospy.Time(0)
    t = np.array([])
    abs_pose = np.array(None)
    manip1 = np.array([])
    manip2 = np.array([])
    computed_alpha = np.array([])
    relative_error = np.array(None)
    relative_error_angle = np.array([])
    va = np.array(None)
    qnorm = 0


def feedbackCb(m):
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, init_time, relative_error, relative_error_angle, va, qnorm

    if init_time == rospy.Time(0):
        init_time = rospy.Time.now()

    qnorm = m.feedback.joint_space_norm

    t = np.append(t, [(rospy.Time.now() - init_time).to_sec()])
    if abs_pose.any() == None:
        abs_pose = np.array([[m.feedback.absolute_position.x,
                              m.feedback.absolute_position.y,
                              m.feedback.absolute_position.z]])
    else:
        abs_pose = np.append(abs_pose, [[m.feedback.absolute_position.x,
                                         m.feedback.absolute_position.y,
                                         m.feedback.absolute_position.z]], axis=0)

    if relative_error.any() == None:
        relative_error = np.array(
            [[m.feedback.relative_error.x, m.feedback.relative_error.y, m.feedback.relative_error.z]])
    else:
        relative_error = np.append(relative_error, [
            [m.feedback.relative_error.x, m.feedback.relative_error.y, m.feedback.relative_error.z]], axis=0)

    if va.any() == None:
        va = np.array([[m.feedback.va.linear.x, m.feedback.va.linear.y, m.feedback.va.linear.z,
                        m.feedback.va.angular.x, m.feedback.va.angular.y, m.feedback.va.angular.z]])
    else:
        va = np.append(va, [[m.feedback.va.linear.x, m.feedback.va.linear.y, m.feedback.va.linear.z,
                             m.feedback.va.angular.x, m.feedback.va.angular.y, m.feedback.va.angular.z]], axis=0)

    relative_error_angle = np.append(
        relative_error_angle, [m.feedback.relative_error_angle])
    manip1 = np.append(manip1, [m.feedback.manip1])
    manip2 = np.append(manip2, [m.feedback.manip2])
    computed_alpha = np.append(computed_alpha, [m.feedback.curr_alpha])


def saveFig(dir, name, fig=1):
    """Save the fig in the images directory."""
    path = dir + name + ".svg"
    f = plt.figure(fig)
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
    if iter >= len(init_obj_frames[case]):
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


def makeFirstCasePlot(iter, color='k', title=None):
    """Should plot the absolute trajectory of a system run."""
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, va
    matplotlib.rcParams['figure.figsize'] = (18, 9)
    fig = plt.figure(iter)
    # ax = fig.gca(projection='3d')
    #
    # ax.scatter(abs_pose[1, 0], abs_pose[1, 1],
    #            abs_pose[1, 2], edgecolors='k', s=100, marker='o', facecolors='w')
    # ax.plot(abs_pose[1:, 0], abs_pose[1:, 1],
    #         abs_pose[1:, 2], color=color, label=lbl)
    # ax.scatter(abs_pose[-1, 0], abs_pose[-1, 1],
    #            abs_pose[-1, 2], c='k', s=100, marker='x')
    # ax.set_xlabel('x [m]')
    # ax.set_ylabel('y [m]')
    # ax.set_zlabel('z [m]')
    # ax.legend()
    plt.subplot(211)
    plt.ylim(-0.05, 0.18)
    plt.plot(t[1:], va[1:, 0], color="k", label="$v_x$")
    plt.plot(t[1:], va[1:, 1], color="teal", label="$v_y$")
    plt.plot(t[1:], va[1:, 2], color="darkorange", label="$v_z$")
    plt.ylabel("Linear velocity [m/s]")
    plt.legend()
    plt.title(title)
    plt.subplot(212)
    plt.ylim(-0.15, 0.4)
    plt.plot(t[1:], va[1:, 3], color="k", label=r"$\omega_x$")
    plt.plot(t[1:], va[1:, 4], color="teal", label=r"$\omega_y$")
    plt.plot(t[1:], va[1:, 5], color="darkorange", label=r"$\omega_z$")
    plt.ylabel("Angular velocity [rad/s]")
    plt.xlabel("Time [s]")
    plt.legend()


def makeSecondCasePlot(color='k', lbl=None):
    """Plot the effective degree of sharing."""
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    matplotlib.rcParams['figure.figsize'] = (18, 9)
    fig = plt.figure(1)
    # plt.plot(t[1:], effective_alpha[1:], color=color, label=lbl)
    plt.xlabel('Time [s]')
    plt.ylabel('Effective ' + r'$\alpha$')


def getDir():
    """Get image dir."""
    global log_directory_name
    rospack = rospkg.RosPack()

    now = datetime.datetime.now()
    path_name = rospack.get_path("coordination_experiments") + "/images/" + str(
        now.month) + str(now.day) + str(now.hour) + str(now.minute) + str(now.second) + "/"

    if not os.path.exists(path_name):
        os.makedirs(path_name)

    return path_name


def makeErrorPlot(color='k', lbl=True, line='-'):
    global t, relative_error, relative_error_angle
    matplotlib.rcParams['figure.figsize'] = (18, 9)
    fig = plt.figure(1)
    plt.subplot(211)
    if lbl:
        plt.plot(t[1:], relative_error[1:, 0],
                 color="k", label=r"$v_x$", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 1],
                 color="teal", label=r"$v_y$", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 2],
                 color="orange", label=r"$v_z$", linestyle=line)
    else:
        plt.plot(t[1:], relative_error[1:, 0],
                 color="k", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 1],
                 color="teal", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 2],
                 color="orange", linestyle=line)
    plt.ylabel("Error [m]")
    plt.title("Linear relative error")
    plt.legend()
    plt.subplot(212)
    plt.plot(t[1:], relative_error_angle[1:],
             color="k", linestyle=line)

    plt.ylabel("Angle [rad]")
    plt.xlabel("Time [m]")
    plt.title("Angular relative error")
    plt.legend()


def makeThirdCasePlot(c_alpha='k', c_mu1='c', c_mu2='darkorange', line='-', lbl=None):
    """Plot the execution alpha and the manipulability indices."""
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha
    matplotlib.rcParams['figure.figsize'] = (9, 7)
    fig = plt.figure(1)
    plt.subplot(211)
    plt.plot(t[2:], computed_alpha[2:], color=c_alpha, label=lbl)
    plt.ylabel(r'$\alpha$')
    plt.title(r"$\alpha$ Evolution")
    plt.legend()
    plt.subplot(212)
    plt.title(r"Manipulability measures")
    plt.plot(t[2:], manip1[2:], color=c_mu1, linestyle=line, label=r"$\mu_1$")
    plt.plot(t[2:], manip2[2:], color=c_mu2, linestyle=line, label=r"$\mu_2$")
    plt.xlabel('Time [s]')
    plt.legend()


if __name__ == "__main__":
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, qnorm
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
                     CoordinationControllerFeedback, feedbackCb, queue_size=1)

    gray = (0.85, 0.87, 0.89)
    matplotlib.rcParams['font.size'] = 22
    matplotlib.rcParams['lines.linewidth'] = 4
    matplotlib.rcParams['figure.subplot.wspace'] = 0.4
    matplotlib.rcParams['figure.subplot.hspace'] = 0.35
    feedback = RunSimulationFeedback()

    rospy.loginfo("Waiting for coordination action server...")
    coordination_client.wait_for_server()

    while not rospy.is_shutdown():
        while not (top_server.is_new_goal_available() or rospy.is_shutdown()):
            rospy.loginfo_throttle(
                60, "Run simulations action server waiting for goal...")
            rospy.sleep(0.5)

        dir = getDir()
        if rospy.is_shutdown():
            sys.exit()

        goal = top_server.accept_new_goal()
        coordination_goal = CoordinationControllerGoal()
        coordination_goal.max_time = 10.0
        coordination_goal.alpha = 0.5
        coordination_goal.limits.position_max_limits.x = 10.6
        coordination_goal.limits.position_max_limits.y = 10.2
        coordination_goal.limits.position_max_limits.z = 10.3
        coordination_goal.limits.position_min_limits.x = -10.4
        coordination_goal.limits.position_min_limits.y = -10.2
        coordination_goal.limits.position_min_limits.z = -10.1
        coordination_goal.limits.orientation_limit_angle = 10.5

        # try:
        #     resp = object_client(False)
        # except rospy.ServiceException, e:
        #     rospy.logerr("Failed to contact the object server: %s" % e)
        #     top_server.set_aborted()
        #     continue

        try:
            resp = sim_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to contact the simulation: %s" % e)
            top_server.set_aborted()
            continue

        success = True
        # TEST CASE I
        if goal.case_one:
            for i in range(len(init_obj_frames['I'])):
                setManipulationTargets('I', i, list)
                iter = 1
                for alpha in (2, 5, 8):
                    resetVars()
                    coordination_goal.alpha = alpha / 10.
                    coordination_goal.dynamic_alpha = False
                    # run relative Jacobian (master-slave)
                    feedback.feedback = "ECTS with alpha = " + str(alpha / 10.)
                    top_server.publish_feedback(feedback)
                    coordination_goal.control_mode.controller = coordination_goal.control_mode.ECTS

                    success = monitor_action_goal(
                        top_server,
                        coordination_client,
                        coordination_goal,
                        action_name=coordination_action_name)

                    if not success:
                        top_server.set_aborted()
                        break

                    rospy.logwarn("JOINT SPACE NORM: %.2f" % qnorm)
                    makeFirstCasePlot(
                        iter, 'k', r"Absolute motion (ECTS), $\alpha = " + str(alpha / 10.) + "$")
                    saveFig(dir, "abs_motion_ects_" +
                            str(alpha / 10.) + "_" + str(i), iter)

                    coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                    resetVars()
                    feedback.feedback = "ERelJac with alpha = " + \
                        str(alpha / 10.)
                    top_server.publish_feedback(feedback)
                    success = monitor_action_goal(
                        top_server,
                        coordination_client,
                        coordination_goal,
                        action_name=coordination_action_name)

                    if not success:
                        top_server.set_aborted()
                        break

                    rospy.logwarn("JOINT SPACE NORM: %.2f" % qnorm)
                    makeFirstCasePlot(
                        iter + 1, "k", r"Absolute motion (relative Jacobian), $\alpha = " + str(alpha / 10.) + "$")

                    saveFig(dir, "abs_motion_reljac_" +
                            str(alpha / 10.) + "_" + str(i), iter + 1)
                    plt.show()
                    iter += 2
                if not success:
                    break

                try:
                    resp = sim_reset()
                except rospy.ServiceException, e:
                    rospy.logerr("Failed to contact the simulation: %s" % e)
                    top_server.set_aborted()
                    continue

        coordination_goal.max_time = 10.0
        # TEST CASE II
        if goal.case_two and success:
            for i in range(len(init_obj_frames['II'])):
                resetVars()
                try:
                    resp = sim_reset()
                except rospy.ServiceException, e:
                    rospy.logerr("Failed to contact the simulation: %s" % e)
                    top_server.set_aborted()
                    continue
                setManipulationTargets('II', i, list)

                # run proposed jacobian without nullspace projection
                coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                coordination_goal.alpha = 0.8
                coordination_goal.use_asymmetric_l_only = True

                success = monitor_action_goal(
                    top_server, coordination_client, coordination_goal, action_name=coordination_action_name)

                if not success:
                    break

                makeErrorPlot(
                    'k', True)
                resetVars()

                coordination_goal.use_asymmetric_l_only = False
                success = monitor_action_goal(
                    top_server, coordination_client, coordination_goal, action_name=coordination_action_name)

                if not success:
                    break

                makeErrorPlot(
                    'grey', False, '--')
                saveFig(dir, "asymmetric_only_" + str(i))
                plt.show()

                # run relative Jacobian (absolute limits)
                # coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJACABSLIM
                # coordination_goal.limits.position_max_limits.x = 0.3
                # coordination_goal.limits.position_max_limits.y = 0.2
                # coordination_goal.limits.position_max_limits.z = 0.3
                # coordination_goal.limits.position_min_limits.x = 0.0
                # coordination_goal.limits.position_min_limits.y = -0.2
                # coordination_goal.limits.position_min_limits.z = -0.1
                # coordination_goal.limits.orientation_limit_angle = 0.4
                # coordination_goal.symmetric_secundary_task = False
                #
                # success = monitor_action_goal(
                #     top_server,
                #     coordination_client,
                #     coordination_goal,
                #     action_name=coordination_action_name)
                #
                # if not success:
                #     break
                #
                # makeSecondCasePlot('k')
                #
                # # rospy.loginfo(
                # #     "Initializing relative Jacobian (symmetric absolute limits)")
                # # coordination_goal.symmetric_secundary_task = True
                # #
                # # success = monitor_action_goal(
                # #     top_server,
                # #     coordination_client,
                # #     coordination_goal,
                # #     action_name=coordination_action_name)
                #
                # # run extended relative Jacobian
                # resetVars()
                # rospy.loginfo("Initializing extended relative Jacobian")
                # coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                # coordination_goal.alpha = 0.9
                # coordination_goal.dynamic_alpha = False
                # success = monitor_action_goal(
                #     top_server,
                #     coordination_client,
                #     coordination_goal,
                #     action_name=coordination_action_name)
                #
                # if not success:
                #     break
                #
                # makeSecondCasePlot('b')
                # plt.show()
                # try:
                #     resp = sim_reset()
                # except rospy.ServiceException, e:
                #     rospy.logerr("Failed to contact the simulation: %s" % e)
                #     top_server.set_aborted()
                #     continue

        # TEST CASE III
        if goal.case_three and success:
            for i in range(len(init_obj_frames['III'])):
                try:
                    resp = sim_reset()
                except rospy.ServiceException, e:
                    rospy.logerr("Failed to contact the simulation: %s" % e)
                    top_server.set_aborted()
                    continue

                resetVars()
                setManipulationTargets('III', i, list)
                # coordination_goal.control_mode.controller = coordination_goal.control_mode.RELJACABSLIM
                # coordination_goal.limits.position_max_limits.x = 10.3
                # coordination_goal.limits.position_max_limits.y = 10.2
                # coordination_goal.limits.position_max_limits.z = 10.3
                # coordination_goal.limits.position_min_limits.x = -10.0
                # coordination_goal.limits.position_min_limits.y = -10.2
                # coordination_goal.limits.position_min_limits.z = -10.1
                # coordination_goal.limits.orientation_limit_angle = 10.5
                # coordination_goal.symmetric_secundary_task = False
                #
                # coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                # coordination_goal.alpha = 0.5
                # coordination_goal.dynamic_alpha = False
                # feedback.feedback = "ERelJac without dynamic alpha"
                # top_server.publish_feedback(feedback)
                # success = monitor_action_goal(
                #     top_server,
                #     coordination_client,
                #     coordination_goal,
                #     action_name=coordination_action_name)
                #
                # if not success:
                #     break

                # makeThirdCasePlot('w', 'grey', 'darkblue')

                resetVars()
                coordination_goal.control_mode.controller = coordination_goal.control_mode.EXTRELJAC
                coordination_goal.alpha = 0.5
                coordination_goal.dynamic_alpha = True
                feedback.feedback = "ERelJac with dynamic alpha"
                top_server.publish_feedback(feedback)
                success = monitor_action_goal(
                    top_server,
                    coordination_client,
                    coordination_goal,
                    action_name=coordination_action_name)

                if not success:
                    break

                makeThirdCasePlot('k', 'grey', 'darkblue',
                                  '-', lbl=r"$\alpha = \frac{\mu_2}{\mu_1 + \mu_2}$")
                saveFig(dir, "dynamic_alpha_" + str(i))
                plt.show()

        if success:
            top_server.set_succeeded()
