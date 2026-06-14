#!/usr/bin/env python

# Test File for PF
# Matt Schmittle
# DO NOT EDIT

import time

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt

import mushr_pf.utils as utils

error_array = []
pf_array = []
true_array = []
started = False
plotted = False


def callback(true_pose, pf_pose):
    print("in callback")
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
    if time.time() - start_time > 40 and not plotted:
        plt.xlabel("x")
        plt.ylabel("y")
        plt.plot(np.array(true_array)[:, 0], np.array(true_array)[:, 1], c="r")
        plt.plot(np.array(pf_array)[:, 0], np.array(pf_array)[:, 1], c="g")
        plt.show()
        plotted = True


def main(args=None):
    rclpy.init()
    node = Node("testpy")
    node.declare_parameter('gt_topic', "/car/pf/inferred_pose")
    node.declare_parameter('plot', True)
    gt_topic = str(node.get_parameter("gt_topic").get_parameter_value().string_value)
    plot = bool(node.get_parameter("plot").get_parameter_value().bool_value)

    pose_sub = message_filters.Subscriber(node, PoseStamped, "/car/car_pose")
    pf_sub = message_filters.Subscriber(node, PoseStamped, gt_topic)

    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, pf_sub], 10, 0.5)
    ts.registerCallback(callback)
    rclpy.spin(node)


main()