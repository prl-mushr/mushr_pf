#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import queue
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan

from mushr_pf import utils
from mushr_pf.motion_model import KinematicMotionModel
from mushr_pf.resample import ReSampler
from mushr_pf.sensor_model import SensorModel

MAP_TOPIC = "/map"


class ParticleFilter(Node):
    """
    Implements particle filtering for estimating the state of the robot car
    """

    def __init__(self):
        """
        Initializes the particle filter
          publish_tf: Whether or not to publish the tf. Should be false in sim, true on real robot
          n_particles: The number of particles
          n_viz_particles: The number of particles to visualize
          odometry_topic: The topic containing odometry information
          motor_state_topic: The topic containing motor state information
          servo_state_topic: The topic containing servo state information
          scan_topic: The topic containing laser scans
          laser_ray_step: Step for downsampling laser scans
          exclude_max_range_rays: Whether to exclude rays that are beyond the max range
          max_range_meters: The max range of the laser
          speed_to_erpm_offset: Offset conversion param from rpm to speed
          speed_to_erpm_gain: Gain conversion param from rpm to speed
          steering_angle_to_servo_offset: Offset conversion param from servo position to steering angle
          steering_angle_to_servo_gain: Gain conversion param from servo position to steering angle
          car_length: The length of the car
        """
        super().__init__("particle_filter")
        
        self.get_logger().info("Initializing particle filter node")

        # Declare parameters
        self.declare_parameter('car_name', "car")
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('n_particles', 1000)
        self.declare_parameter('n_viz_particles', 60)
        self.declare_parameter('odometry_topic', '/vesc/odom')
        self.declare_parameter('motor_state_topic', '/vesc/sensors/core')
        self.declare_parameter('servo_state_topic', '/vesc/sensors/servo_position_command')
        self.declare_parameter('scan_topic', '/car/scan')
        self.declare_parameter('laser_ray_step', 18)
        self.declare_parameter('exclude_max_range_rays', True)
        self.declare_parameter('max_range_meters', 11.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('speed_to_erpm_gain', 4350)
        self.declare_parameter('steering_angle_to_servo_offset', 0.5)
        self.declare_parameter('steering_angle_to_servo_gain', -1.2135)
        self.declare_parameter('car_length', 0.33)
        
        # Get parameters
        car_name = self.get_parameter('car_name').value
        publish_tf = self.get_parameter('publish_tf').value
        n_particles = self.get_parameter('n_particles').value
        n_viz_particles = self.get_parameter('n_viz_particles').value
        odometry_topic = self.get_parameter('odometry_topic').value
        motor_state_topic = self.get_parameter('motor_state_topic').value
        servo_state_topic = self.get_parameter('servo_state_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        laser_ray_step = self.get_parameter('laser_ray_step').value
        exclude_max_range_rays = self.get_parameter('exclude_max_range_rays').value
        max_range_meters = self.get_parameter('max_range_meters').value
        speed_to_erpm_offset = self.get_parameter('speed_to_erpm_offset').value
        speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        steering_angle_to_servo_offset = self.get_parameter('steering_angle_to_servo_offset').value
        steering_angle_to_servo_gain = self.get_parameter('steering_angle_to_servo_gain').value
        self.car_length = self.get_parameter('car_length').value


        self.PUBLISH_TF = publish_tf
        # The number of particles in this implementation, the total number of particles is constant.
        self.N_PARTICLES = n_particles
        self.N_VIZ_PARTICLES = n_viz_particles  # The number of particles to visualize

        # Cached list of particle indices
        self.particle_indices = np.arange(self.N_PARTICLES)
        # Numpy matrix of dimension N_PARTICLES x 3
        self.particles = np.zeros((self.N_PARTICLES, 3))
        # Numpy matrix containing weight for each particle
        self.weights = np.ones(self.N_PARTICLES) / float(self.N_PARTICLES)

        # Name of car
        self.name = car_name

        # A lock used to prevent concurrency issues. You do not need to worry about this
        self.state_lock = Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("waiting on map message")

        self.permissible_region = None
        self.map_info = None


        self.map_service_name = '/map_server/map'

        self.map_client = self.create_client(GetMap, self.map_service_name)

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map Server not available, waiting again...')

        def get_map():
            req = GetMap.Request()
            future = self.map_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                map_msg = future.result().map  # full nav_msgs/OccupancyGrid

                array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
                permissible_region = np.zeros_like(array_255, dtype=bool)
                permissible_region[array_255 == 0] = 1

                return permissible_region, map_msg.info, map_msg
            else:
                self.get_logger().error('Service call failed %r' % (future.exception(),))
                return None, None
                        
        # Get the map
        self.permissible_region, self.map_info, self.raw_map_msg = get_map()

        # Globally initialize the particles

        # Publish particle filter state
        # Used to create a tf between the map and the laser for visualization
        self.pub_tf = tf2_ros.TransformBroadcaster(self)

        # Publishes the expected pose
        self.pose_pub = self.create_publisher(PoseStamped, "~/inferred_pose", 1)
        # Publishes a subsample of the particles
        self.particle_pub = self.create_publisher(PoseArray, "~/particles", 1)
        # Publishes the most recent laser scan
        self.pub_laser = self.create_publisher(LaserScan, "~/scan", 1)
        # Publishes the path of the car
        self.pub_odom = self.create_publisher(Odometry, "~/odom", 1)

        self.initialize_global()

        # An object used for resampling
        self.resampler = ReSampler(self.particles, self.weights, self.state_lock)

        # An object used for applying sensor model
        self.sensor_model = SensorModel(self,
            scan_topic,
            laser_ray_step,
            exclude_max_range_rays,
            max_range_meters,
            self.raw_map_msg,
            self.particles,
            self.weights,
            self.car_length,
            self.state_lock,
        )

        # An object used for applying kinematic motion model
        self.motion_model = KinematicMotionModel(self,
            motor_state_topic,
            servo_state_topic,
            speed_to_erpm_offset,
            speed_to_erpm_gain,
            steering_angle_to_servo_offset,
            steering_angle_to_servo_gain,
            self.car_length,
            self.particles,
            self.state_lock,
        )

        self.permissible_x, self.permissible_y = np.where(self.permissible_region == 1)

        # Parameters/flags/vars for global localization
        self.global_localize = False
        self.global_suspend = False
        self.ents = None
        self.ents_sum = 0.0
        self.noisy_cnt = 0
        # number of regions to partition. Simulation: 25, Real: 5.
        self.NUM_REGIONS = 25
        # number of updates for regional localization. Simulation 5, Real: 3.
        self.REGIONAL_ROUNDS = 5
        self.regions = []
        self.debug_mode = False
        if self.debug_mode:
            self.global_localize = True  # True when doing global localization

        # precompute regions. Each region is represented by arrays of x, y indices on the map
        region_size = int(len(self.permissible_x) / self.NUM_REGIONS)
        idx = np.argsort(self.permissible_y)  # column-major
        _px, _py = self.permissible_x[idx], self.permissible_y[idx]
        for i in range(self.NUM_REGIONS):
            self.regions.append(
                (
                    _px[i * region_size : (i + 1) * region_size],
                    _py[i * region_size : (i + 1) * region_size],
                )
            )

        # Subscribe to the '/clicked_point' topic. Publised by Foxglove. 
        # See clicked_pose_cb function in this file for more info
        self.click_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_cb, 1
        )

        self.last_scan_msg = None

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos_profile=1)

        while self.last_scan_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Initialization complete")

    def _scan_callback(self, msg):
        """Callback to capture first laser scan"""
        if self.last_scan_msg is None:
            self.last_scan_msg = msg

    def initialize_global(self):
        """
        Initialize the particles to cover the map
        """
        self.state_lock.acquire()
        # Get in-bounds locations
        permissible_x, permissible_y = np.where(self.permissible_region == 1)

        # The number of particles at each location, each with different rotation
        angle_step = 4

        # The sample interval for permissible states
        permissible_step = angle_step * int(len(permissible_x) / self.particles.shape[0])

        # Indices of permissible states to use
        indices = np.arange(0, len(permissible_x), permissible_step)[
            : int((self.particles.shape[0] / angle_step))
        ]

        # Proxy for the new particles
        permissible_states = np.zeros((self.particles.shape[0], 3))

        # Loop through permissible states, each iteration drawing particles with
        # different rotation
        for i in range(angle_step):
            idx_start = i * int(self.particles.shape[0] / angle_step)
            idx_end = (i + 1) * int(self.particles.shape[0] / angle_step)

            permissible_states[idx_start:idx_end, 0] = permissible_y[indices]
            permissible_states[idx_start:idx_end, 1] = permissible_x[indices]
            permissible_states[idx_start:idx_end, 2] = i * (2 * np.pi / angle_step)

        # Transform permissible states to be w.r.t world
        utils.map_to_world(permissible_states, self.map_info)

        # Reset particles and weights
        self.particles[:, :] = permissible_states[:, :]
        self.weights[:] = 1.0 / self.particles.shape[0]
        self.state_lock.release()

    def reinit_cb(self, msg):
        if self.debug_mode:
            self.global_localize = True

    def publish_tf(self, pose, stamp=None):
        """
        Publish a tf between the laser and the map
        This is necessary in order to visualize the laser scan within the map
          pose: The pose of the laser w.r.t the map
          stamp: The time at which this pose was calculated, defaults to None - resulting
                 in using the time at which this function was called as the stamp
        """
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        try:
            # Lookup the offset between laser and odom
            transform = self.tf_buffer.lookup_transform(
                self.name + "/laser_link",
                self.name + "/odom",
                rclpy.time.Time(),
            )
            delta_off = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            delta_rot = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )

            # Transform offset to be w.r.t the map
            off_x = delta_off[0] * np.cos(pose[2]) - delta_off[1] * np.sin(pose[2])
            off_y = delta_off[0] * np.sin(pose[2]) + delta_off[1] * np.cos(pose[2])

            # Broadcast the tf
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = "/map"
            tf_msg.child_frame_id = self.name + "/odom"
            tf_msg.transform.translation.x = pose[0] + off_x
            tf_msg.transform.translation.y = pose[1] + off_y
            tf_msg.transform.translation.z = 0.0
            quat = quaternion_from_euler(
                0, 0, pose[2] + euler_from_quaternion(delta_rot)[2]
            )
            tf_msg.transform.rotation.x = quat[0]
            tf_msg.transform.rotation.y = quat[1]
            tf_msg.transform.rotation.z = quat[2]
            tf_msg.transform.rotation.w = quat[3]
            self.pub_tf.sendTransform(tf_msg)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warning(f"failed to find odom transform: {e}")

    def expected_pose(self):
        """
        Uses cosine and sine averaging to more accurately compute average theta
        To get one combined value use the dot product of position and weight vectors
        https://en.wikipedia.org/wiki/Mean_of_circular_quantities

        returns: np array of the expected pose given the current particles and weights
        """
        cosines = np.cos(self.particles[:, 2])
        sines = np.sin(self.particles[:, 2])
        theta = np.arctan2(np.dot(sines, self.weights), np.dot(cosines, self.weights))
        position = np.dot(self.particles[:, 0:2].transpose(), self.weights)
        position[0] += (self.car_length / 2) * np.cos(theta)
        position[1] += (self.car_length / 2) * np.sin(theta)
        return np.array((position[0], position[1], theta), dtype=float)

    def clicked_point_cb(self, msg):
        """
        Reinitialize particles and weights according to the received initial pose
        Applies Gaussian noise to each particle's pose

        msg: PointStamped 
        returns: nothing
        """
        self.state_lock.acquire()
        point = msg.point
        print("SETTING POSE")

        VAR_X = 0.001
        VAR_Y = 0.001
        VAR_THETA = 0.001
        theta = 0.0 
        x = point.x
        y = point.y
        self.particles[:, 0] = np.random.normal(x, VAR_X, self.particles.shape[0])
        self.particles[:, 1] = np.random.normal(y, VAR_Y, self.particles.shape[0])
        self.particles[:, 2] = np.random.normal(
            theta, VAR_THETA, self.particles.shape[0]
        )
        self.weights.fill(1 / self.N_PARTICLES)
        self.state_lock.release()

    def visualize(self):
        """
        Visualize the current state of the filter
           (1) Publishes a tf between the map and the laser. Necessary for visualizing the laser scan in the map
           (2) Publishes the most recent laser measurement. Note that the frame_id of this message should be '/laser'
           (3) Publishes a PoseStamped message indicating the expected pose of the car
           (4) Publishes a subsample of the particles (use self.N_VIZ_PARTICLES).
               Sample so that particles with higher weights are more likely to be sampled.
        """
        self.state_lock.acquire()
        self.inferred_pose = self.expected_pose()

        if isinstance(self.inferred_pose, np.ndarray):
            if self.PUBLISH_TF:
                self.publish_tf(self.inferred_pose)
            ps = PoseStamped()
            ps.header = utils.make_header("map", self.get_clock().now().to_msg())
            ps.pose.position.x = self.inferred_pose[0]
            ps.pose.position.y = self.inferred_pose[1]
            ps.pose.orientation = utils.angle_to_quaternion(self.inferred_pose[2])
            if self.pose_pub.get_subscription_count() > 0:
                self.pose_pub.publish(ps)
            if self.pub_odom.get_subscription_count() > 0:
                odom = Odometry()
                odom.header = ps.header
                odom.pose.pose = ps.pose
                self.pub_odom.publish(odom)

        if self.particle_pub.get_subscription_count() > 0:
            if self.particles.shape[0] > self.N_VIZ_PARTICLES:
                # randomly downsample particles
                proposal_indices = np.random.choice(
                    self.particle_indices, self.N_VIZ_PARTICLES, p=self.weights
                )
                self.publish_particles(self.particles[proposal_indices, :])
            else:
                self.publish_particles(self.particles)

        if self.pub_laser.get_subscription_count() > 0 and isinstance(
            self.sensor_model.last_laser, LaserScan
        ):
            self.sensor_model.last_laser.header.frame_id = "/laser"
            self.sensor_model.last_laser.header.stamp = self.get_clock().now().to_msg()
            self.pub_laser.publish(self.sensor_model.last_laser)
        self.state_lock.release()

    def publish_particles(self, particles):
        """
        Helper function for publishing a pose array of particles
          particles: To particles to publish
        """
        pa = PoseArray()
        pa.header = utils.make_header("map", self.get_clock().now().to_msg())
        pa.poses = utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def set_particles(self, region):
        self.state_lock.acquire()
        # Get in-bounds locations
        permissible_x, permissible_y = region
        assert len(permissible_x) >= self.particles.shape[0]

        # The number of particles at each location, each with different rotation
        angle_step = 4
        # The sample interval for permissible states
        permissible_step = angle_step * len(permissible_x) / self.particles.shape[0]
        # Indices of permissible states to use
        indices = np.arange(0, len(permissible_x), permissible_step)[
            : (self.particles.shape[0] / angle_step)
        ]
        # Proxy for the new particles
        permissible_states = np.zeros((self.particles.shape[0], 3))

        # Loop through permissible states, each iteration drawing particles with
        # different rotation
        for i in range(angle_step):
            idx_start = i * (self.particles.shape[0] / angle_step)
            idx_end = (i + 1) * (self.particles.shape[0] / angle_step)

            permissible_states[idx_start:idx_end, 0] = permissible_y[indices]
            permissible_states[idx_start:idx_end, 1] = permissible_x[indices]
            permissible_states[idx_start:idx_end, 2] = i * (2 * np.pi / angle_step)

        # Transform permissible states to be w.r.t world
        utils.map_to_world(permissible_states, self.map_info)

        # Reset particles and weights
        self.particles[:, :] = permissible_states[:, :]
        self.weights[:] = 1.0 / self.particles.shape[0]
        self.publish_particles(self.particles)
        self.state_lock.release()

    def global_localization(self):
        self.sensor_model.reset_confidence()

        candidate_num = self.particles.shape[0] / self.NUM_REGIONS
        regional_particles = []
        regional_weights = []

        for i in range(self.NUM_REGIONS):
            self.set_particles(self.regions[i])
            cnt_updates = 0
            while cnt_updates < self.REGIONAL_ROUNDS:  # each cluster update
                if self.sensor_model.do_resample:  # if weights updated by sensor model
                    self.resampler.resample_low_variance()
                    self.visualize()
                    self.sensor_model.do_resample = False
                    cnt_updates += 1
            self.state_lock.acquire()
            candidate_indices = np.argsort(self.weights)[-candidate_num:]
            candidates = self.particles[candidate_indices]
            candidates_weights = self.weights[candidate_indices]
            regional_particles.append(candidates.copy())  # save the particles
            regional_weights.append(candidates_weights)  # save the particles' weights
            self.state_lock.release()

        self.state_lock.acquire()
        self.particles[:] = np.concatenate(regional_particles)
        self.weights[:] = np.concatenate(regional_weights)
        self.weights /= self.weights.sum()
        self.global_localize = False
        self.global_suspend = True
        self.sensor_model.do_confidence_update = True
        self.state_lock.release()

    def suspend_update(self):
        self.state_lock.acquire()
        ent = -((self.weights * np.log2(self.weights)).sum())
        print("entropy ==", ent)
        self.ents_sum += ent
        self.ents.put(ent)
        if self.ents.qsize() > 10:
            self.ents_sum -= self.ents.get()
        self.state_lock.release()
        if self.ents_sum / 10 >= 8:
            self.global_suspend = False
        elif 2.0 < ent < 3.0:
            self.resampler.resample_low_variance()


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()

    try:
        while rclpy.ok():
            rclpy.spin_once(pf, timeout_sec=0.1)

            if pf.sensor_model.confidence < 1e-20 and not pf.global_localize:
                pf.get_logger().info("=================== KIDNAPPED =====================")
                pf.global_localize = True

            if pf.global_localize:
                temp = pf.N_VIZ_PARTICLES
                pf.N_VIZ_PARTICLES = 1000
                pf.global_localization()
                pf.visualize()
                pf.N_VIZ_PARTICLES = temp
                pf.ents = queue.Queue()
                pf.ents_sum = 0.0
                pf.noisy_cnt = 0
            elif pf.sensor_model.do_resample:
                pf.sensor_model.do_resample = False
                pf.resampler.resample_low_variance()
                pf.visualize()
    finally:
        pf.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
