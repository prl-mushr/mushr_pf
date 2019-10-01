#!/usr/bin/env python

# Test File for PF
# Matt Schmittle
# DO NOT EDIT

import time

import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt

import mushr_pf.utils as utils

error_array = []
pf_array = []
true_array = []
started = False
plotted = False


def callback(true_pose, pf_pose):
    global error_array, start_time, started, plotted
    if not started:
        start_time = time.time()
        started = True
    error_x = (true_pose.pose.position.x - pf_pose.pose.position.x) ** 2
    error_y = (true_pose.pose.position.y - pf_pose.pose.position.y) ** 2
    error_t = (
        utils.quaternion_to_angle(true_pose.pose.orientation)
        - utils.quaternion_to_angle(pf_pose.pose.orientation)
    ) ** 2
    error_array.append([error_x, error_y, error_t])
    print_array = np.array(error_array)
    print(
        "Median X Error:",
        np.median(print_array[:, 0]),
        "Median Y Error:",
        np.median(print_array[:, 1]),
        "Median Theta Error:",
        np.median(print_array[:, 2]),
    )
    true_array.append([true_pose.pose.position.x, true_pose.pose.position.y])
    pf_array.append([pf_pose.pose.position.x, pf_pose.pose.position.y])

    # Plot trajectories
    if plot and time.time() - start_time > 40 and not plotted:
        plt.xlabel("x")
        plt.ylabel("y")
        plt.plot(np.array(true_array)[:, 0], np.array(true_array)[:, 1], c="r")
        plt.plot(np.array(pf_array)[:, 0], np.array(pf_array)[:, 1], c="g")
        plt.show()
        plotted = True


if __name__ == "__main__":
    rospy.init_node("testpf", anonymous=True)
    gt_topic = str(rospy.get_param("~gt_topic", "/pf/ta/viz/inferred_pose"))
    plot = bool(rospy.get_param("~plot", False))

    # Time Synchronizer
    pose_sub = message_filters.Subscriber("/pf/viz/inferred_pose", PoseStamped)
    pf_sub = message_filters.Subscriber(gt_topic, PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, pf_sub], 10, 0.1)
    ts.registerCallback(callback)
    rospy.spin()
