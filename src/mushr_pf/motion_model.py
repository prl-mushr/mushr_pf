#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from threading import Lock

import numpy as np
import rospy
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

# Tune these Values!
KM_V_NOISE = 0.4  # Kinematic car velocity noise std dev
KM_DELTA_NOISE = 0.2  # Kinematic car delta noise std dev
KM_X_FIX_NOISE = 3e-2  # Kinematic car x position constant noise std dev
KM_Y_FIX_NOISE = 3e-2  # Kinematic car y position constant noise std dev
KM_THETA_FIX_NOISE = 1e-1  # Kinematic car theta constant noise std dev


# #Tune these Values!
# KM_V_NOISE = 0.01             # Kinematic car velocity noise std dev
# KM_DELTA_NOISE = 0.06          # Kinematic car delta noise std dev
# KM_X_FIX_NOISE = 3e-2          # Kinematic car x position constant noise std dev
# KM_Y_FIX_NOISE = 1e-3        # Kinematic car y position constant noise std dev
# KM_THETA_FIX_NOISE = 1e-2      # Kinematic car theta constant noise std dev

# #Tune these Values!
# KM_V_NOISE = 0.015            # Kinematic car velocity noise std dev
# KM_DELTA_NOISE = 0.065             # Kinematic car delta noise std dev
# KM_X_FIX_NOISE = 1e-2          # Kinematic car x position constant noise std dev
# KM_Y_FIX_NOISE = 1e-2        # Kinematic car y position constant noise std dev
# KM_THETA_FIX_NOISE = 1e-2      # Kinematic car theta constant noise std dev

"""
  Propagates the particles forward based the velocity and steering angle of the car
"""


class KinematicMotionModel:

    """
    Initializes the kinematic motion model
      motor_state_topic: The topic containing motor state information
      servo_state_topic: The topic containing servo state information
      speed_to_erpm_offset: Offset conversion param from rpm to speed
      speed_to_erpm_gain: Gain conversion param from rpm to speed
      steering_angle_to_servo_offset: Offset conversion param from servo position to steering angle
      steering_angle_to_servo_gain: Gain conversion param from servo position to steering angle
      car_length: The length of the car
      particles: The particles to propagate forward
      state_lock: Controls access to particles
    """

    def __init__(
        self,
        motor_state_topic,
        servo_state_topic,
        speed_to_erpm_offset,
        speed_to_erpm_gain,
        steering_to_servo_offset,
        steering_to_servo_gain,
        car_length,
        particles,
        state_lock=None,
    ):
        self.last_servo_cmd = None  # The most recent servo command
        self.last_vesc_stamp = None  # The time stamp from the previous vesc state msg
        self.particles = particles
        self.SPEED_TO_ERPM_OFFSET = (
            speed_to_erpm_offset  # Offset conversion param from rpm to speed
        )
        self.SPEED_TO_ERPM_GAIN = (
            speed_to_erpm_gain  # Gain conversion param from rpm to speed
        )
        self.STEERING_TO_SERVO_OFFSET = steering_to_servo_offset  # Offset conversion param from servo position to steering angle
        self.STEERING_TO_SERVO_GAIN = steering_to_servo_gain  # Gain conversion param from servo position to steering angle
        self.CAR_LENGTH = car_length  # The length of the car

        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        # This subscriber just caches the most recent servo position command
        self.servo_pos_sub = rospy.Subscriber(
            servo_state_topic, Float64, self.servo_cb, queue_size=1
        )
        # Subscribe to the state of the vesc
        self.motion_sub = rospy.Subscriber(
            motor_state_topic, VescStateStamped, self.motion_cb, queue_size=1
        )

    """
    Caches the most recent servo command
      msg: A std_msgs/Float64 message
  """

    def servo_cb(self, msg):
        self.last_servo_cmd = msg.data  # Update servo command

    """
    Converts messages to controls and applies the kinematic car model to the
    particles
      msg: a vesc_msgs/VescStateStamped message
  """

    def motion_cb(self, msg):
        self.state_lock.acquire()
        if self.last_servo_cmd is None:
            self.state_lock.release()
            return

        if self.last_vesc_stamp is None:
            print("Vesc callback called for first time....")
            self.last_vesc_stamp = msg.header.stamp
            self.state_lock.release()
            return

        # Convert raw msgs to controls
        # Note that control = (raw_msg_val - offset_param) / gain_param
        curr_speed = (
            msg.state.speed - self.SPEED_TO_ERPM_OFFSET
        ) / self.SPEED_TO_ERPM_GAIN

        curr_steering_angle = (
            self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET
        ) / self.STEERING_TO_SERVO_GAIN
        dt = (msg.header.stamp - self.last_vesc_stamp).to_sec()

        # Propagate particles forward in place
        self.apply_motion_model(
            proposal_dist=self.particles, control=[curr_speed, curr_steering_angle, dt]
        )

        self.last_vesc_stamp = msg.header.stamp
        self.state_lock.release()

    def apply_motion_model(self, proposal_dist, control):
        """
        Propagates particles forward (in-place) by applying the kinematic model and adding
        sampled gaussian noise
        proposal_dist: The particles to propagate
        control: List containing velocity, steering angle, and timer interval - [v,delta,dt]
        returns: nothing
        """
        # Separate control
        v, delta, dt = control

        # Add control noise
        v = np.random.normal(loc=v, scale=KM_V_NOISE, size=proposal_dist[:, 0].shape)
        delta = np.random.normal(
            loc=delta, scale=KM_DELTA_NOISE, size=proposal_dist[:, 0].shape
        )

        # apply motion model's update rule
        theta = proposal_dist[:, 2]
        theta_new = theta + v / self.CAR_LENGTH * np.tan(delta) * dt
        # x
        proposal_dist[:, 0] += (
            self.CAR_LENGTH / np.tan(delta) * (np.sin(theta_new) - np.sin(theta))
        )
        # y
        proposal_dist[:, 1] += (
            self.CAR_LENGTH / np.tan(delta) * (-np.cos(theta_new) + np.cos(theta))
        )

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
            loc=theta_new, scale=KM_THETA_FIX_NOISE, size=proposal_dist[:, 2].shape
        )
        # print 'v: %f, delta: %f, x: %f, y: %f, theta: %f'%(np.mean(v), np.mean(delta), np.mean(proposal_dist[:,0]), np.mean(proposal_dist[:,1]), np.mean(proposal_dist[:,2]))

        # Limit particle rotation to be between -pi and pi
        proposal_dist[proposal_dist[:, 2] < -1 * np.pi, 2] += 2 * np.pi
        proposal_dist[proposal_dist[:, 2] > np.pi, 2] -= 2 * np.pi
