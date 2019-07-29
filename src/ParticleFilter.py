#!/usr/bin/env python

import rospy 
import numpy as np
import time
import utils as Utils
import tf.transformations
import tf
from threading import Lock

from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry,OccupancyGrid

import math
import Queue
from ReSample import ReSampler
from SensorModel import SensorModel
from MotionModel import  KinematicMotionModel

MAP_TOPIC = "/map"
PUBLISH_PREFIX = '/pf'
PUBLISH_TF = True

'''
  Implements particle filtering for estimating the state of the robot car
'''
class ParticleFilter():


  '''
  Initializes the particle filter
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
  '''
  def __init__(self, n_particles, n_viz_particles, odometry_topic,
               motor_state_topic, servo_state_topic, scan_topic, laser_ray_step,
               exclude_max_range_rays, max_range_meters, 
               speed_to_erpm_offset, speed_to_erpm_gain, steering_angle_to_servo_offset,
               steering_angle_to_servo_gain, car_length):
    self.N_PARTICLES = n_particles # The number of particles
                                   # In this implementation, the total number of 
                                   # particles is constant
    self.N_VIZ_PARTICLES = n_viz_particles # The number of particles to visualize

    self.particle_indices = np.arange(self.N_PARTICLES) # Cached list of particle indices
    self.particles = np.zeros((self.N_PARTICLES,3)) # Numpy matrix of dimension N_PARTICLES x 3
    self.weights = np.ones(self.N_PARTICLES) / float(self.N_PARTICLES) # Numpy matrix containing weight for each particle

    self.state_lock = Lock() # A lock used to prevent concurrency issues. You do not need to worry about this
    
    self.tfl = tf.TransformListener() # Transforms points between coordinate frames

    # Get the map
    map_msg = rospy.wait_for_message(MAP_TOPIC,OccupancyGrid)
    self.map_info = map_msg.info # Save info about map for later use    

    # Create numpy array representing map for later use
    array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    self.permissible_region = np.zeros_like(array_255, dtype=bool)
    self.permissible_region[array_255==0] = 1 # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
                                              # With values 0: not permissible, 1: permissible    

    # Globally initialize the particles
    
    # Publish particle filter state
    self.pub_tf = tf.TransformBroadcaster() # Used to create a tf between the map and the laser for visualization    
    self.pose_pub      = rospy.Publisher(PUBLISH_PREFIX + "/inferred_pose", PoseStamped, queue_size = 1) # Publishes the expected pose
    self.particle_pub  = rospy.Publisher(PUBLISH_PREFIX + "/particles", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    self.pub_laser     = rospy.Publisher(PUBLISH_PREFIX + "/scan", LaserScan, queue_size = 1) # Publishes the most recent laser scan
    self.pub_odom      = rospy.Publisher(PUBLISH_PREFIX + "/odom", Odometry, queue_size = 1) # Publishes the path of the car
    
    rospy.sleep(1.0)
    self.initialize_global()
    
    self.resampler = ReSampler(self.particles, self.weights, self.state_lock)  # An object used for resampling

    # An object used for applying sensor model
    self.sensor_model = SensorModel(scan_topic, laser_ray_step, exclude_max_range_rays, 
                                    max_range_meters, map_msg, self.particles, self.weights,car_length,
                                    self.state_lock)
    
    # An object used for applying kinematic motion model
    self.motion_model = KinematicMotionModel(motor_state_topic, servo_state_topic, 
          speed_to_erpm_offset, speed_to_erpm_gain, 
          steering_angle_to_servo_offset, steering_angle_to_servo_gain, 
          car_length, self.particles, self.state_lock) 

    self.permissible_x, self.permissible_y = np.where(self.permissible_region == 1)
    
    # Parameters/flags/vars for global localization
    self.global_localize = False
    self.global_suspend = False
    self.ents = None
    self.ents_sum = 0.0
    self.noisy_cnt = 0
    self.NUM_REGIONS = 25     # number of regions to partition. Simulation: 25, Real: 5.
    self.REGIONAL_ROUNDS = 5  # number of updates for regional localization. Simulation 5, Real: 3.
    self.regions = []
    self.click_mode = True
    self.debug_mode = False
    if self.debug_mode:
        self.global_localize = True     # True when doing global localization

    # precompute regions. Each region is represented by arrays of x, y indices on the map
    region_size = len(self.permissible_x) / self.NUM_REGIONS
    # for i in xrange(self.NUM_REGIONS):  # row-major
    #   self.regions.append((self.permissible_x[i*region_size:(i+1)*region_size],
    #                        self.permissible_y[i*region_size:(i+1)*region_size]))
    idx = np.argsort(self.permissible_y)  # column-major
    _px, _py = self.permissible_x[idx], self.permissible_y[idx]
    for i in xrange(self.NUM_REGIONS):
      self.regions.append((_px[i*region_size:(i+1)*region_size],
                           _py[i*region_size:(i+1)*region_size]))


    # Subscribe to the '/initialpose' topic. Publised by RVIZ. See clicked_pose_cb function in this file for more info
    # three different reactions to click on rviz
    self.click_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose_cb, queue_size=1)
    self.init_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.reinit_cb, queue_size=1)

    rospy.wait_for_message(scan_topic, LaserScan)
    print('Initialization complete')


  '''
    Initialize the particles to cover the map
  '''
  def initialize_global(self):
    self.state_lock.acquire()
    # Get in-bounds locations
    permissible_x, permissible_y = np.where(self.permissible_region == 1)


    angle_step = 4 # The number of particles at each location, each with different rotation
    permissible_step = angle_step*len(permissible_x)/self.particles.shape[0] # The sample interval for permissible states
    indices = np.arange(0, len(permissible_x), permissible_step)[:(self.particles.shape[0]/angle_step)] # Indices of permissible states to use
    permissible_states = np.zeros((self.particles.shape[0],3)) # Proxy for the new particles

    # Loop through permissible states, each iteration drawing particles with
    # different rotation
    for i in xrange(angle_step):
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),0] = permissible_y[indices]
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),1] = permissible_x[indices]
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),2] = i*(2*np.pi / angle_step) 

    # Transform permissible states to be w.r.t world
    Utils.map_to_world(permissible_states, self.map_info)

    # Reset particles and weights
    self.particles[:,:] = permissible_states[:,:]
    self.weights[:] = 1.0 / self.particles.shape[0]
    #self.publish_particles(self.particles)
    self.state_lock.release()

  def reinit_cb(self, msg):
    if self.debug_mode:
        self.global_localize = True

  '''
    Publish a tf between the laser and the map
    This is necessary in order to visualize the laser scan within the map
      pose: The pose of the laser w.r.t the map
      stamp: The time at which this pose was calculated, defaults to None - resulting
             in using the time at which this function was called as the stamp
  '''
  def publish_tf(self,pose,stamp=None):
    if stamp is None:
        stamp = rospy.Time.now()
    try:
      # Lookup the offset between laser and odom  
      delta_off, delta_rot = self.tfl.lookupTransform("/laser_link","/odom",rospy.Time(0))

      # Transform offset to be w.r.t the map
      off_x = delta_off[0]*np.cos(pose[2]) - delta_off[1]*np.sin(pose[2])
      off_y = delta_off[0]*np.sin(pose[2]) + delta_off[1]*np.cos(pose[2])

      # Broadcast the tf
      self.pub_tf.sendTransform((pose[0]+off_x,pose[1]+off_y,0.0),tf.transformations.quaternion_from_euler(0,0,pose[2]+tf.transformations.euler_from_quaternion(delta_rot)[2]),stamp,"/map","/odom")

    except (tf.LookupException) as e: # Will occur if odom frame does not exist
      print(e)
      print("failed to find odom")
      #self.pub_tf.sendTransform((pose[0],pose[1],0),tf.transformations.quaternion_from_euler(0,0,pose[2]), stamp , "/laser_link", "/map")

  '''
    Uses cosine and sine averaging to more accurately compute average theta
    To get one combined value use the dot product of position and weight vectors
    https://en.wikipedia.org/wiki/Mean_of_circular_quantities

    returns: np array of the expected pose given the current particles and weights
  '''
  def expected_pose(self):
        cosines = np.cos(self.particles[:,2])
        sines = np.sin(self.particles[:,2])
        theta = np.arctan2(np.dot(sines,self.weights),np.dot(cosines, self.weights))
        position = np.dot(self.particles[:,0:2].transpose(), self.weights)
        position[0] += (car_length/2)*np.cos(theta)
        position[1] += (car_length/2)*np.sin(theta)
        return np.array((position[0], position[1], theta),dtype=np.float)

  '''
    Students implement (add tip about vectorized stuff)
    Reinitialize particles and weights according to the received initial pose
    Applies Gaussian noise to each particle's pose
    HINT: use Utils.quaternion_to_angle()
    Remember to use vectorized computation!

    msg: '/initialpose' topic. RVIZ publishes a message to this topic when you specify an initial pose using its GUI
    returns: nothing
  '''
  def clicked_pose_cb(self, msg):
    if self.click_mode:
        self.state_lock.acquire()
        pose = msg.pose.pose
        print "SETTING POSE"

        #YOUR CODE HERE
        VAR_X = 0.001
        VAR_Y = 0.001
        VAR_THETA = 0.001
        quat = pose.orientation
        theta = Utils.quaternion_to_angle(quat)
        x = pose.position.x
        y = pose.position.y
        self.particles[:,0] = np.random.normal(x, VAR_X, self.particles.shape[0])
        self.particles[:,1] = np.random.normal(y, VAR_Y, self.particles.shape[0])
        self.particles[:,2] = np.random.normal(theta, VAR_THETA, self.particles.shape[0])
        self.weights.fill(1 / self.N_PARTICLES)
        self.state_lock.release()

  '''
    Visualize the current state of the filter
   (1) Publishes a tf between the map and the laser. Necessary for visualizing the laser scan in the map
   (2) Publishes the most recent laser measurement. Note that the frame_id of this message should be '/laser'
   (3) Publishes a PoseStamped message indicating the expected pose of the car
   (4) Publishes a subsample of the particles (use self.N_VIZ_PARTICLES). 
       Sample so that particles with higher weights are more likely to be sampled.
  '''
  def visualize(self):
    #print 'Visualizing...'
    self.state_lock.acquire()
    self.inferred_pose = self.expected_pose()

    if isinstance(self.inferred_pose, np.ndarray):
      if PUBLISH_TF:
        self.publish_tf(self.inferred_pose)
      ps = PoseStamped()
      ps.header = Utils.make_header("map")
      ps.pose.position.x = self.inferred_pose[0]
      ps.pose.position.y = self.inferred_pose[1]
      ps.pose.orientation = Utils.angle_to_quaternion(self.inferred_pose[2])
      if(self.pose_pub.get_num_connections() > 0):
        self.pose_pub.publish(ps)
      if(self.pub_odom.get_num_connections() > 0):
        odom = Odometry()
        odom.header = ps.header
        odom.pose.pose = ps.pose
        self.pub_odom.publish(odom)

    if self.particle_pub.get_num_connections() > 0:
      if self.particles.shape[0] > self.N_VIZ_PARTICLES:
        # randomly downsample particles
        proposal_indices = np.random.choice(self.particle_indices, self.N_VIZ_PARTICLES, p=self.weights)
        # proposal_indices = np.random.choice(self.particle_indices, self.N_VIZ_PARTICLES)
        self.publish_particles(self.particles[proposal_indices,:])
      else:
        self.publish_particles(self.particles)

    if self.pub_laser.get_num_connections() > 0 and isinstance(self.sensor_model.last_laser, LaserScan):
      self.sensor_model.last_laser.header.frame_id = "/laser"
      self.sensor_model.last_laser.header.stamp = rospy.Time.now()
      self.pub_laser.publish(self.sensor_model.last_laser)
    self.state_lock.release()

  '''
  Helper function for publishing a pose array of particles
    particles: To particles to publish
  '''
  def publish_particles(self, particles):
    pa = PoseArray()
    pa.header = Utils.make_header("map")
    pa.poses = Utils.particles_to_poses(particles)
    self.particle_pub.publish(pa)

  def set_particles(self, region):
    self.state_lock.acquire()
    # Get in-bounds locations
    permissible_x, permissible_y = region
    assert len(permissible_x) >= self.particles.shape[0]
    
    angle_step = 4 # The number of particles at each location, each with different rotation
    permissible_step = angle_step*len(permissible_x)/self.particles.shape[0] # The sample interval for permissible states
    indices = np.arange(0, len(permissible_x), permissible_step)[:(self.particles.shape[0]/angle_step)] # Indices of permissible states to use
    permissible_states = np.zeros((self.particles.shape[0],3)) # Proxy for the new particles
    
    # Loop through permissible states, each iteration drawing particles with
    # different rotation
    for i in xrange(angle_step):
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),0] = permissible_y[indices]
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),1] = permissible_x[indices]
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),2] = i*(2*np.pi / angle_step) 
     
    # Transform permissible states to be w.r.t world
    Utils.map_to_world(permissible_states, self.map_info)
    
    # Reset particles and weights
    self.particles[:,:] = permissible_states[:,:]
    self.weights[:] = 1.0 / self.particles.shape[0]
    self.publish_particles(self.particles)
    self.state_lock.release()


  def global_localization(self):
    self.sensor_model.reset_confidence()

    candidate_num = self.particles.shape[0] / self.NUM_REGIONS
    regional_particles = []
    regional_weights = []

    for i in xrange(self.NUM_REGIONS):
      self.set_particles(self.regions[i])
      cnt_updates = 0
      while cnt_updates < self.REGIONAL_ROUNDS: # each cluster update 
          if self.sensor_model.do_resample:  # if weights updated by sensor model
              self.resampler.resample_low_variance()
              self.visualize()
              self.sensor_model.do_resample = False
              cnt_updates += 1
      self.state_lock.acquire()
      candidate_indices = np.argsort(self.weights)[-candidate_num:]
      candidates = self.particles[candidate_indices]
      candidates_weights = self.weights[candidate_indices] 
      regional_particles.append(candidates.copy()) # save the particles
      regional_weights.append(candidates_weights)  # save the particles' weights
      self.state_lock.release()

    self.state_lock.acquire()
    self.particles[:] = np.concatenate(regional_particles)
    self.weights[:] = np.concatenate(regional_weights)
    self.weights /= self.weights.sum()
    #self.weights[:] = 1.0 / self.particles.shape[0]
    self.global_localize = False
    self.global_suspend = True
    self.sensor_model.do_confidence_update = True
    self.state_lock.release()

  def suspend_update(self):
    self.state_lock.acquire()
    ent = -((self.weights*np.log2(self.weights)).sum())
    print 'entropy ==', ent
    self.ents_sum += ent
    self.ents.put(ent)
    if self.ents.qsize() > 10:
        self.ents_sum -= self.ents.get()
    self.state_lock.release()
    if self.ents_sum / 10 >= 8:
        self.global_suspend = False
    elif 2.0 < ent < 3.0:
        self.resampler.resample_low_variance()


# Suggested main 
if __name__ == '__main__':
  rospy.init_node("particle_filter", anonymous=True) # Initialize the node
  
  n_particles = int(rospy.get_param("~n_particles")) # The number of particles
  n_viz_particles = int(rospy.get_param("~n_viz_particles")) # The number of particles to visualize
  odometry_topic = rospy.get_param("~odometry_topic", "/vesc/odom") # The topic containing odometry information
  motor_state_topic = rospy.get_param("~motor_state_topic", "/vesc/sensors/core") # The topic containing motor state information
  servo_state_topic = rospy.get_param("~servo_state_topic", "/vesc/sensors/servo_position_command") # The topic containing servo state information
  scan_topic = rospy.get_param("~scan_topic", "/scan") # The topic containing laser scans
  laser_ray_step = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
  exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays")) # Whether to exclude rays that are beyond the max range
  max_range_meters = float(rospy.get_param("~max_range_meters")) # The max range of the laser

  speed_to_erpm_offset = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0)) # Offset conversion param from rpm to speed
  speed_to_erpm_gain = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4350))   # Gain conversion param from rpm to speed
  steering_angle_to_servo_offset = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5)) # Offset conversion param from servo position to steering angle
  steering_angle_to_servo_gain = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135)) # Gain conversion param from servo position to steering angle    
  car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33)) # The length of the car
  
  # Create the particle filter  
  pf = ParticleFilter(n_particles, n_viz_particles, odometry_topic,
                      motor_state_topic, servo_state_topic, scan_topic, laser_ray_step,
                      exclude_max_range_rays, max_range_meters, 
                      speed_to_erpm_offset, speed_to_erpm_gain, steering_angle_to_servo_offset,
                      steering_angle_to_servo_gain, car_length)
  
  while not rospy.is_shutdown():  # Keep going until we kill it
    # Callbacks are running in separate threads
    
    if pf.sensor_model.confidence < 1e-20 and not pf.global_localize:
        print '=================== KIDNAPPED ====================='
        pf.global_localize = True

    # update particle filter
    if pf.global_localize: # no resample
        temp = pf.N_VIZ_PARTICLES
        pf.N_VIZ_PARTICLES = 1000
        pf.global_localization()
        pf.visualize()
        pf.N_VIZ_PARTICLES = temp
        pf.ents = Queue.Queue()
        pf.ents_sum = 0.0
        pf.noisy_cnt = 0

    elif pf.sensor_model.do_resample: # Check if the sensor model says it's time to resample
      pf.sensor_model.do_resample = False # Reset so that we don't keep resampling
      pf.resampler.resample_low_variance()
      pf.visualize() # Perform visualization

