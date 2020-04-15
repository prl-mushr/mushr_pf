#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from threading import Lock

import numpy as np
import rospy
from utils import quaternion_to_angle, shortest_angle
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry

# Tune these Values!
KM_X_FIX_NOISE = 3e-3  # Camera delta x constant noise std dev
KM_Y_FIX_NOISE = 3e-3  # Camera delta y constant noise std dev
KM_THETA_FIX_NOISE = 1e-2  # Camera delta theta constant noise std dev

#KM_X_FIX_NOISE = 3e-2  # Camera delta x constant noise std dev
#KM_Y_FIX_NOISE = 3e-2  # Camera delta y constant noise std dev
#KM_THETA_FIX_NOISE = 1e-1  # Camera delta theta constant noise std dev


"""
  Propagates the particles forward based on the odometry of the camera
"""


class CameraMotionModel:

    """
    Initializes the kinematic motion model
      camera_odom_topic: 
      # Add a transform for the camera and the car base at some point  in the future
      particles: The particles to propagate forward
      state_lock: Controls access to particles
    """

    def __init__(
        self,
        camera_odom_topic,
        particles,
        state_lock=None,
    ):
        self.prev_odom = None  # The previous odometry reading
        self.curr_odom = None  # The current odometry reading
        self.particles = particles

        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        # This subscriber just caches the current odometry reading
        self.cam_sub = rospy.Subscriber(
            camera_odom_topic, Odometry, self.cam_cb, queue_size=1
        )

    """
    Caches the most recent odometry reading and applies motion model update
      msg: a nav_msgs/Odometry message
    """
    def cam_cb(self, msg):
        self.state_lock.acquire()
        self.curr_odom = msg

        # Fence-posting for first iteration
        if self.prev_odom is None:
            print("Camera callback called for first time....")
            self.prev_odom = self.curr_odom
            self.state_lock.release()
            return
        
        self.apply_motion_model(proposal_dist=self.particles)
        self.prev_odom = self.curr_odom
        self.state_lock.release()


    def apply_motion_model(self, proposal_dist):
        """
          Propagates particles forward (in-place) by applying the last known camera displacement
          and adding sampled gaussian noise
            proposal_dist: The particles to propagate
            returns: nothing
         """
        # Get displacement from prev Iteration
        delta_x = self.curr_odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        delta_y = self.curr_odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        curr_theta = quaternion_to_angle(self.curr_odom.pose.pose.orientation)
        prev_theta = quaternion_to_angle(self.prev_odom.pose.pose.orientation)
        delta_theta = shortest_angle(curr_theta, prev_theta)

        
        # Update all particles
        proposal_dist[:, 0] += delta_x
        proposal_dist[:, 1] += delta_y
        proposal_dist[:, 2] += delta_theta

        
        # Add noise
        proposal_dist[:, 0] = np.random.normal(
            loc=proposal_dist[:, 0],
            scale=KM_X_FIX_NOISE,
            size=proposal_dist[:, 0].shape,
        )
        proposal_dist[:, 1] = np.random.normal(
            loc=proposal_dist[:, 1],
            scale=KM_Y_FIX_NOISE,
            size=proposal_dist[:, 1].shape,
        )
        proposal_dist[:, 2] = np.random.normal(
            loc=proposal_dist[:, 2],
            scale=KM_THETA_FIX_NOISE,
            size=proposal_dist[:, 2].shape,
        )
        # print 'v: %f, delta: %f, x: %f, y: %f, theta: %f'%(np.mean(v), np.mean(delta), np.mean(proposal_dist[:,0]), np.mean(proposal_dist[:,1]), np.mean(proposal_dist[:,2]))

        # Limit particle rotation to be between -pi and pi
        proposal_dist[proposal_dist[:, 2] < -1 * np.pi, 2] += 2 * np.pi
        proposal_dist[proposal_dist[:, 2] > np.pi, 2] -= 2 * np.pi
