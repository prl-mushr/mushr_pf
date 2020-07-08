#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import Queue
from threading import Lock

import numpy as np
import range_libc
import rospy
from sensor_msgs.msg import LaserScan

THETA_DISCRETIZATION = 112  # Discretization of scanning angle
INV_SQUASH_FACTOR = 0.2  # Factor for helping the weight distribution to be less peaked

Z_SHORT = 0.1  # Weight for short reading
Z_MAX = 0.05  # Weight for max reading
Z_RAND = 0.05  # Weight for random reading
SIGMA_HIT = 8.0  # Noise value for hit reading
Z_HIT = 0.80  # Weight for hit reading

"""
  Weights particles according to their agreement with the observed data, but using camera odometry
"""


class CameraSensorModel:
    def __init__(
        self,
        cam_topic,
        particles,
        weights,
        car_length,
        state_lock=None,
    ):

        """
          Initializes the sensor model
            cam_topic: The topic containing camera_odom messages
            particles: The particles to be weighted
            weights: The weights of the particles
            state_lock: Used to control access to particles and weights
        """
        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        self.particles = particles
        self.weights = weights

        self.CAR_LENGTH = car_length

        self.prev_odom = None  # The previous odometry reading                                       
        self.curr_odom = None  # The current odometry reading           

        # Load the sensor model expressed as a table
        self.range_method.set_sensor_model(self.precompute_sensor_model(max_range_px))
        self.queries = None
        self.ranges = None
        self.laser_angles = None  # The angles of each ray
        self.downsampled_angles = None  # The angles of the downsampled rays

        # Set so that outside code can know that it's time to resample
        self.do_resample = False

        # Subscribe to camera
        self.cam_sub = rospy.Subscriber(
            camera_odom_topic, Odometry, self.cam_cb, queue_size=1
        )



        # FOR KIDNAPPED ROBOT PROBLEM
        self.do_confidence_update = False
        self.CONF_HISTORY_SIZE = 10
        self.conf_history = Queue.Queue()
        self.conf_sum = 0.0
        self.confidence = 1.0

    def reset_confidence(self):
        self.do_confidence_update = False
        self.conf_history = Queue.Queue()
        self.conf_sum = 0.0
        self.confidence = 1.0

    """                                                                                              
    Caches the most recent odometry reading and applies motion model update                          
      msg: a nav_msgs/Odometry message                                                               
    """
    def cam_cb(self, msg):
        self.state_lock.acquire()
        self.curr_odom = msg

        # Fence-posting for first iteration                                                          
        if self.prev_odom is None:
            print("camera_mm: Camera callback called for first time....")
            self.prev_odom = self.curr_odom
            self.state_lock.release()
            return

        # Try to update the cam to map trasform (needed for displacement vector direction)           
        try:
            (trans, self.rotation) = self.tf_listener.lookupTransform(
                '/map', '/camera_odom_frame', rospy.Time(0)
            )
        except:
            print "camera_mm: No tf  published"

        self.apply_motion_model(proposal_dist=self.particles)
        self.prev_odom = self.curr_odom
        self.state_lock.release()


    def get_expected_position(self, proposal_dist):
        """                                                                                        
        I want this to get the expected new inferred position, then apply a distribution like a gaussian to reweight particles based on this.
          Propagates particles forward (in-place) by applying the last known camera displacement    
          and adding sampled gaussian noise.
            proposal_dist: The particles to propagate                                               
            returns: nothing                                                                        
         """
        # Get displacement from prev Iteration                                                      
        delta_x = self.curr_odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        delta_y = self.curr_odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        curr_theta = quaternion_to_angle(self.curr_odom.pose.pose.orientation)
        prev_theta = quaternion_to_angle(self.prev_odom.pose.pose.orientation)
        delta_theta = shortest_angle(curr_theta, prev_theta)

        # Transform the displacement vector onto map frame using cam to map tf                      
        rot_matrix = R.from_quat(self.rotation)
        delta_x, delta_y, delta_z = rot_matrix.apply((delta_x, delta_y, 0))
#        print self.rotation, delta_x, delta_y, delta_z                                             

        # Display magnitude of displacement vector                                                  
#        vector_mag = np.linalg.norm(np.array([delta_x, delta_y]))                                  
#        print vector_mag                                                                            

        # Convert percent error to a scaling factor for displacement vector                          
        magnitude_scale = np.random.normal(
            loc=1, scale=CAM_MAG_NOISE_PERCENT, size=proposal_dist[:, 0].shape
        )

        # Update all particles with magnitude noise                                                  
        proposal_dist[:, 0] += magnitude_scale * delta_x
        proposal_dist[:, 1] += magnitude_scale * delta_y
        proposal_dist[:, 2] += delta_theta

        # Add noise                                                                                  
        proposal_dist[:, 0] = np.random.normal(
            loc=proposal_dist[:, 0],
            scale=CAM_X_FIX_NOISE,
            size=proposal_dist[:, 0].shape,
        )
        proposal_dist[:, 1] = np.random.normal(
            loc=proposal_dist[:, 1],
            scale=CAM_Y_FIX_NOISE,
            size=proposal_dist[:, 1].shape,
        )
        proposal_dist[:, 2] = np.random.normal(
            loc=proposal_dist[:, 2],
            scale=CAM_THETA_FIX_NOISE,
            size=proposal_dist[:, 2].shape,
        )

        # print 'v: %f, delta: %f, x: %f, y: %f, theta: %f'%(np.mean(v), np.mean(delta), np.mean(proposal_dist[:,0]), np.mean(proposal_dist[:,1]), np.mean(proposal_dist[:,2]))                       

        # Limit particle rotation to be between -pi and pi                                         
        proposal_dist[proposal_dist[:, 2] < -1 * np.pi, 2] += 2 * np.pi
        proposal_dist[proposal_dist[:, 2] > np.pi, 2] -= 2 * np.pi


    def laser_cb(self, msg):
        """
        Infers the expected offset Downsamples laser measurements and applies sensor model
          msg: A sensor_msgs/LaserScan
        """
        self.state_lock.acquire()

        # Down sample the laser rays
        if not self.EXCLUDE_MAX_RANGE_RAYS:
            # Initialize angle arrays
            if not isinstance(self.laser_angles, np.ndarray):
                self.laser_angles = np.linspace(
                    msg.angle_min, msg.angle_max, len(msg.ranges)
                )
                self.downsampled_angles = np.copy(
                    self.laser_angles[0 :: self.LASER_RAY_STEP]
                ).astype(np.float32)

            self.downsampled_ranges = np.array(
                msg.ranges[:: self.LASER_RAY_STEP]
            )  # Down sample
            self.downsampled_ranges[
                np.isnan(self.downsampled_ranges)
            ] = self.MAX_RANGE_METERS  # Remove nans
            self.downsampled_ranges[
                self.downsampled_ranges[:] == 0
            ] = self.MAX_RANGE_METERS  # Remove 0 values
        else:
            # Initialize angle array
            if not isinstance(self.laser_angles, np.ndarray):
                self.laser_angles = np.linspace(
                    msg.angle_min, msg.angle_max, len(msg.ranges)
                )
            ranges = np.array(msg.ranges)  # Get the measurements
            ranges[np.isnan(ranges)] = self.MAX_RANGE_METERS  # Remove nans
            # Find non-extreme measurements
            valid_indices = np.logical_and(
                ranges > 0.01, ranges < self.MAX_RANGE_METERS
            )
            # Get angles corresponding to non-extreme measurements
            self.filtered_angles = np.copy(self.laser_angles[valid_indices]).astype(
                np.float32
            )
            # Get non-extreme measurements
            self.filtered_ranges = np.copy(ranges[valid_indices]).astype(np.float32)

            # Compute expected number of rays
            ray_count = int(self.laser_angles.shape[0] / self.LASER_RAY_STEP)
            # Get downsample indices
            sample_indices = np.arange(
                0,
                self.filtered_angles.shape[0],
                float(self.filtered_angles.shape[0]) / ray_count,
            ).astype(np.int)

            # Initialize down sample angles
            self.downsampled_angles = np.zeros(ray_count + 1, dtype=np.float32)
            # Initialize down sample measurements
            self.downsampled_ranges = np.zeros(ray_count + 1, dtype=np.float32)

            # Populate downsample angles
            self.downsampled_angles[: sample_indices.shape[0]] = np.copy(
                self.filtered_angles[sample_indices]
            )
            # Populate downsample measurements
            self.downsampled_ranges[: sample_indices.shape[0]] = np.copy(
                self.filtered_ranges[sample_indices]
            )

        # Compute the observation
        # obs is a a two element tuple
        # obs[0] is the downsampled ranges
        # obs[1] is the downsampled angles
        # Each element of obs must be a numpy array of type np.float32
        # Use self.LASER_RAY_STEP as the downsampling step
        # Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
        obs = (
            np.copy(self.downsampled_ranges).astype(np.float32),
            self.downsampled_angles.astype(np.float32),
        )

        self.apply_sensor_model(self.particles, obs, self.weights)
        # print 'sensor model stats', np.mean(self.weights), np.var(self.weights)

        # FOR KIDNAPPED ROBOT PROBLEM
        # simple idea of comparing one sensor reading
        curr_weight = np.mean(self.weights)
        if self.do_confidence_update:
            self.conf_sum += curr_weight
            self.conf_history.put(curr_weight)
            if self.conf_history.qsize() > self.CONF_HISTORY_SIZE:
                self.conf_sum -= self.conf_history.get()
            if self.conf_history.qsize() == self.CONF_HISTORY_SIZE:
                self.confidence = self.conf_sum / self.conf_history.qsize()
            print("confidence:", self.confidence)

        self.weights /= np.sum(self.weights)

        self.last_laser = msg
        self.do_resample = True
        self.state_lock.release()

    def precompute_sensor_model(self, max_range_px):
        """
        Compute table enumerating the probability of observing a measurement
        given the expected measurement
        Element (r,d) of the table is the probability of observing measurement r
        when the expected measurement is d
        max_range_px: The maximum range in pixels
        returns: the table (which is a numpy array with dimensions [max_range_px+1, max_range_px+1])
        """
        table_width = int(max_range_px) + 1

        # Don't forget to normalize the weights!
        def pdf(z_obs, z):
            var = SIGMA_HIT ** 2
            p_hit = (1 / np.sqrt(2 * np.pi * var)) * np.exp(
                -(np.power(z_obs - z, 2) / (2 * var))
            )  # gaussian
            p_short = (
                (z_obs < z) * 2 * (z - z_obs) / (z + 1e-12) / (z + 1e-12)
            )  # linear
            p_rand = 1.0 / max_range_px  # uniform
            p_max = z_obs == max_range_px  # only max
            return Z_HIT * p_hit + Z_SHORT * p_short + Z_MAX * p_max + Z_RAND * p_rand

        table = np.fromfunction(pdf, (table_width, table_width), dtype=np.float32)
        table /= table.sum(axis=0)  # normalize
        return table

    def apply_sensor_model(self, proposal_dist, obs, weights):
        """
        Updates the particle weights in-place based on the observed laser scan
          proposal_dist: The particles
          obs: The most recent observation
          weights: The weights of each particle
        """

        obs_ranges = obs[0]
        obs_angles = obs[1]
        num_rays = obs_angles.shape[0]

        # Only allocate buffers once to avoid slowness
        if not isinstance(self.queries, np.ndarray):
            self.queries = np.zeros((proposal_dist.shape[0], 3), dtype=np.float32)
            self.ranges = np.zeros(num_rays * proposal_dist.shape[0], dtype=np.float32)

        self.queries[:, :] = proposal_dist[:, :]

        self.queries[:, 0] += (self.CAR_LENGTH / 2) * np.cos(self.queries[:, 2])
        self.queries[:, 1] += (self.CAR_LENGTH / 2) * np.sin(self.queries[:, 2])

        # Raycasting to get expected measurements
        self.range_method.calc_range_repeat_angles(
            self.queries, obs_angles, self.ranges
        )

        # Evaluate the sensor model
        self.range_method.eval_sensor_model(
            obs_ranges, self.ranges, weights, num_rays, proposal_dist.shape[0]
        )

        # Squash weights to prevent too much peakiness
        np.power(weights, INV_SQUASH_FACTOR, weights)
