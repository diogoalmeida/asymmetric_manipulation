#!/usr/bin/env python
import datetime
import os
import sys
import time

import actionlib
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np
import rospkg
import rospy
import tf
from asymmetric_manipulation.msg import (CoordinationControllerAction,
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
    'III': [{
        'left': [(0.36, 0.15, 0.36), (0, 0, 0, 1)],
        'right': [(0.508, -0.13, -0.04), (0, 0, 0, 1)]
    }, {
        'left': [(0.445, -0.05, 0.21), (-0.024939, -0.011954, -0.053292, 0.9982)],
        'right': [(0.445, -0.05, 0.21), (-0.023947, -0.0081608, -0.46361, 0.88568)]
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
p1 = np.array(None)
angle1 = np.array([])


def resetVars():
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, init_time, relative_error, relative_error_angle, va, qnorm, p1, angle1
    init_time = rospy.Time(0)
    t = np.array([])
    abs_pose = np.array(None)
    manip1 = np.array([])
    manip2 = np.array([])
    computed_alpha = np.array([])
    relative_error = np.array(None)
    relative_error_angle = np.array([])
    va = np.array(None)
    p1 = np.array(None)
    angle1 = np.array([])
    qnorm = 0


def feedbackCb(m):
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, init_time, relative_error, relative_error_angle, va, qnorm, p1, angle1

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

    if p1.any() == None:
        p1 = np.array([[m.feedback.p1.x, m.feedback.p1.y, m.feedback.p1.z]])
    else:
        p1 = np.append(p1, [[m.feedback.p1.x, m.feedback.p1.y, m.feedback.p1.z]], axis=0)

    relative_error_angle = np.append(relative_error_angle, [m.feedback.relative_error_angle])
    manip1 = np.append(manip1, [m.feedback.manip1])
    manip2 = np.append(manip2, [m.feedback.manip2])
    angle1 = np.append(angle1, [m.feedback.obj1_angle])
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

    rospy.loginfo("Sending object server request")
    resp = obj_server(req)


def makeFirstCasePlot(iter, color='k', title=None):
    """Should plot the absolute trajectory of a system run."""
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, va
    matplotlib.rcParams['figure.figsize'] = (8, 8)
    fig = plt.figure(iter)
    plt.subplot(211)
    plt.ylim(-0.1, 0.18)
    plt.plot(t[1:], va[1:, 0], color="k", label="$v_x$")
    plt.plot(t[1:], va[1:, 1], color="r", label="$v_y$")
    plt.plot(t[1:], va[1:, 2], color="b", label="$v_z$")
    plt.ylabel("Linear [m/s]")
    plt.legend()
    plt.title(title)
    plt.subplot(212)
    plt.ylim(-0.1, 0.4)
    plt.plot(t[1:], va[1:, 3], color="k", label=r"$\omega_x$")
    plt.plot(t[1:], va[1:, 4], color="r", label=r"$\omega_y$")
    plt.plot(t[1:], va[1:, 5], color="b", label=r"$\omega_z$")

    plt.ylabel("Angular [rad/s]")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.tight_layout()


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
    path_name = rospack.get_path("asymmetric_manipulation") + "/images/" + str(
        now.month) + str(now.day) + str(now.hour) + str(now.minute) + str(now.second) + "/"

    if not os.path.exists(path_name):
        os.makedirs(path_name)

    return path_name


def makeErrorPlot(color='k', lbl=True, line='-'):
    global t, relative_error, relative_error_angle
    matplotlib.rcParams['figure.figsize'] = (8, 8)
    fig = plt.figure(1)
    plt.subplot(211)
    if lbl:
        plt.plot(t[1:], relative_error[1:, 0],
                 color="k", label=r"$x$", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 1],
                 color="r", label=r"$y$", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 2],
                 color="b", label=r"$z$", linestyle=line)
    else:
        plt.plot(t[1:], relative_error[1:, 0],
                 color="k", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 1],
                 color="r", linestyle=line)
        plt.plot(t[1:], relative_error[1:, 2],
                 color="b", linestyle=line)
    plt.ylabel("Error [m]")
    plt.title("Linear relative error")
    plt.legend()
    plt.subplot(212)
    plt.plot(t[1:], relative_error_angle[1:],
             color="k", linestyle=line)

    plt.ylabel("Angle [rad]")
    plt.xlabel("Time [s]")
    plt.title("Angular relative error")
    # plt.legend()
    plt.tight_layout()


def makeThirdCasePlot():
    """Plot the transient of the secondary task."""
    global t, p1, angle1
    matplotlib.rcParams['figure.figsize'] = (9, 7)
    fig = plt.figure(1)
    plt.subplot(211)
    plt.plot(t[2:], np.linalg.norm(p1[2] - p1[2:], axis=1))
    plt.ylabel(r'[m]')
    plt.title(r"$|p_d - p_1|$")
    plt.legend()
    plt.subplot(212)
    plt.title(r"$|\vartheta_d - \vartheta_1|$")
    plt.ylabel(r'[rad]')
    plt.plot(t[2:], np.abs(angle1[2] - angle1[2:]))
    plt.xlabel('Time [s]')


def sendCaseTwo(server, client, goal, action_name, plot=False):
    """
        Run case study two experiments.

        This compares the ECTS-based differential IK method with using the extended
        relative Jacobian.

        @param server The experiment server
        @param client The control client (i.e., which simulates the methods).
        @param goal The action goal to the client.
        @param action_name Name of the client's action.

        @returns False in case something fails, true otherwise.
    """
    for i in range(len(init_obj_frames['I'])):
        setManipulationTargets('I', i, list)
        iter = 1
        for alpha in (8, ):
            resetVars()
            goal.alpha = alpha / 10.

            # run relative Jacobian (master-slave)
            feedback.feedback = "ECTS with alpha = " + str(alpha / 10.)
            server.publish_feedback(feedback)
            goal.control_mode.controller = goal.control_mode.ECTS

            success = monitor_action_goal(
                server,
                client,
                goal,
                action_name=action_name)

            if not success:
                server.set_aborted()
                return False

            rospy.logwarn("JOINT SPACE NORM: %.2f" % qnorm)
            makeFirstCasePlot(iter, 'k', r"Absolute motion (ECTS), $\alpha = " + str(alpha / 10.) + "$")
            saveFig(dir, "abs_motion_ects_" + str(alpha / 10.) + "_" + str(i), iter)

            if plot:
                plt.show()
            else:
                plt.close()

            goal.control_mode.controller = goal.control_mode.EXTRELJAC
            resetVars()
            feedback.feedback = "ERelJac with alpha = " + \
                str(alpha / 10.)
            server.publish_feedback(feedback)
            success = monitor_action_goal(
                server,
                client,
                goal,
                action_name=action_name)

            if not success:
                server.set_aborted()
                return False

            rospy.logwarn("JOINT SPACE NORM: %.2f" % qnorm)
            makeFirstCasePlot(iter + 1, "k", r"Absolute motion (Ours), $\alpha = " + str(alpha / 10.) + "$")

            saveFig(dir, "abs_motion_reljac_" + str(alpha / 10.) + "_" + str(i), iter + 1)

            if plot:
                plt.show()
            else:
                plt.close()

            iter += 2
        if not success:
            break

        try:
            resp = sim_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to contact the simulation: %s" % e)
            server.set_aborted()
            return False

    return True


def sendCaseOne(server, client, goal, action_name, plot=False):
    """
        Run case study one.

        In this case study, we illustrate the effects of having asymmetrical
        absolute motion as a functional redundancy of the cooperative manipulation
        system, and show how a projection on the nullspace of the relative Jacobian
        solves this issue.

        @param server The experiment server
        @param client The control client (i.e., which simulates the methods).
        @param goal The action goal to the client.
        @param action_name Name of the client's action.

        @returns False in case something fails, true otherwise.
    """
    for i in range(len(init_obj_frames['II'])):
        resetVars()
        try:
            resp = sim_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to contact the simulation: %s" % e)
            server.set_aborted()
            return False

        rospy.loginfo("Simulation was reset successfully")
        setManipulationTargets('II', i, list)
        rospy.loginfo("Set manipulation targets")

        # run proposed jacobian without nullspace projection
        goal.control_mode.controller = goal.control_mode.EXTRELJAC
        goal.alpha = 0.8
        goal.use_asymmetric_l_only = True

        feedback.feedback = "ERelJac (no nullspace proj) with alpha = " + str(goal.alpha)
        # server.publish_feedback(feedback)

        rospy.loginfo("Sending goal")
        success = monitor_action_goal(
            server, client, goal, action_name=action_name)

        if not success:
            return False

        makeErrorPlot('grey', False, '--')
        resetVars()

        goal.use_asymmetric_l_only = False
        feedback.feedback = "ERelJac with alpha = " + str(goal.alpha)
        # server.publish_feedback(feedback)
        success = monitor_action_goal(
            server, client, goal, action_name=action_name)

        if not success:
            return False

        makeErrorPlot('k', True)
        saveFig(dir, "asymmetric_only_" + str(i))

        if plot:
            plt.show()
        else:
            plt.close()


    return True


def sendCaseThree(server, client, goal, action_name, plot=False):
    """
        Run case study three.

        This depicts the effect of assigning a static pose to one of the end-effectors
        as a secondary task to the relative Jacobian differential IK method. Depending on the
        method's gains, this will have different transients, resulting in asymmetrical motion.
        The result is similar to a master-slave behavior, but with an avoidable degradation of the
        secondary task's performance.

        @param server The experiment server
        @param client The control client (i.e., which simulates the methods).
        @param goal The action goal to the client.
        @param action_name Name of the client's action.

        @returns False in case something fails, true otherwise.
    """
    for i in range(len(init_obj_frames['III'])):
        try:
            resp = sim_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to contact the simulation: %s" % e)
            server.set_aborted()
            continue

        resetVars()
        setManipulationTargets('III', i, list)
        resetVars()

        goal.control_mode.controller = goal.control_mode.RELJAC
        feedback.feedback = "RelJac with secondary task"
        server.publish_feedback(feedback)
        success = monitor_action_goal(
            server,
            client,
            goal,
            action_name=action_name)

        if not success:
            break

        makeThirdCasePlot()
        saveFig(dir, "reljac_masterslave_" + str(i))

        if plot:
            plt.show()

    return True


if __name__ == "__main__":
    global t, abs_pose, effective_alpha, manip1, manip2, computed_alpha, qnorm
    rospy.init_node("run_sims")
    top_server = actionlib.SimpleActionServer("/coordination_simulations/initialize", RunSimulationAction, auto_start=False)
    coordination_action_name = "coordination_controller/coordination_control"
    coordination_client = actionlib.SimpleActionClient(coordination_action_name, CoordinationControllerAction)
    object_server_name = "/object_server/toggle_marker_update"
    object_client = rospy.ServiceProxy(object_server_name, SetBool)
    sim_reset = rospy.ServiceProxy("/state_reset", Empty)
    list = tf.TransformListener()
    rospy.Subscriber(coordination_action_name + "/feedback",
                     CoordinationControllerFeedback, feedbackCb, queue_size=1)

    gray = (0.85, 0.87, 0.89)
    matplotlib.rcParams['font.size'] = 22
    matplotlib.rcParams['lines.linewidth'] = 2
    matplotlib.rcParams['figure.subplot.wspace'] = 0.4
    matplotlib.rcParams['figure.subplot.hspace'] = 0.35
    feedback = RunSimulationFeedback()

    rospy.loginfo("Waiting for coordination action server...")
    coordination_client.wait_for_server()
    top_server.start()

    while not rospy.is_shutdown():
        try:
            while not (top_server.is_new_goal_available() or rospy.is_shutdown()):
                rospy.loginfo_throttle(
                    60, "Run simulations action server waiting for goal...")
                time.sleep(0.5)

            dir = getDir()
            if rospy.is_shutdown():
                sys.exit()

            goal = top_server.accept_new_goal()
            coordination_goal = CoordinationControllerGoal()
            coordination_goal.max_time = 10.0
            coordination_goal.alpha = 0.5
            coordination_goal.use_limits = False
            coordination_goal.dynamic_alpha = False

            try:
                resp = sim_reset()
            except rospy.ServiceException, e:
                rospy.logerr("Failed to contact the simulation: %s" % e)
                top_server.set_aborted()
                continue

            success = True
            rospy.loginfo("New goal acquired!")
            # TEST CASE I
            if goal.case_one:
                success = sendCaseOne(top_server, coordination_client, coordination_goal, coordination_action_name, goal.show_plots)

            coordination_goal.max_time = 10.0

            # TEST CASE II
            if goal.case_two and success:
                success = sendCaseTwo(top_server, coordination_client, coordination_goal, coordination_action_name, goal.show_plots)

            # TEST CASE III
            if goal.example_1 and success:
                success = sendCaseThree(top_server, coordination_client, coordination_goal, coordination_action_name, goal.show_plots)

            if success:
                top_server.set_succeeded()

        except rospy.exceptions.ROSTimeMovedBackwardsException, e:
            rospy.logwarn("Simulation reset detected. Restart action client.")
            if top_server.is_active():
                top_server.set_aborted()

            top_server = actionlib.SimpleActionServer("/coordination_simulations/initialize", RunSimulationAction, auto_start=False)
            top_server.start()
            continue
