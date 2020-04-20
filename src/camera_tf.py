#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR                                                                                                                 
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import roslib
import rospy

import tf
from utils import quaternion_to_angle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped

class CameraTf:
    """                                                                                                                                                                                                    
      Transforms the camera's odometry frame to map such that the current camera odom 
      aligns with given clicked pose estimates. Initialized at map origin.
    """
    
    def __init__(self, cam_odom_topic, cam_frame, map_frame):
        """
          Initializes the camera transform node, and publishes an initial transform
            cam_odom_topic: The topic that camera odometry is published to
            cam_frame: The name of frame of the camera
            map_fram: The name of the frame of the map
        """
        self.cam_odom_topic = cam_odom_topic
        self.cam_frame = cam_frame
        self.map_frame = map_frame

        # Used to hold current camera pose
        self.curr_pose = None

        # Initialize target_pose to origin
        self.target_pose = Pose(Point(0,0,0), tf.transformations.quaternion_from_euler(0,0,0))

        # Transform information, initialized as no transform
        self.translation = (0,0,0)
        self.rotation = tf.transformations.quaternion_from_euler(0,0,0)

        # Subscriber to receive any clicked poses
        self.click_sub = rospy.Subscriber(
            "/initialpose",
            PoseWithCovarianceStamped,
            self.clicked_pose_cb,
            queue_size=1,
        )

        # Subscriber to update camera odometry and publish transform
        self.cam_sb = rospy.Subscriber(
            cam_odom_topic,Odometry, self.send_camera_tf
        )

        # Transform broadcaster
        self.cam_br = tf.TransformBroadcaster()

        # Send an initial transform
        self.cam_br.sendTransform((0,0,0),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  self.cam_frame,
                                  self.map_frame)

    def clicked_pose_cb(self, msg):
        """
          Sets the target pose to be the given 2D pose estimate, then updates the transformation
          based on the new target pose and the current camera pose. 
            msg: geometry_msgs/PoseWithCovarianceStamped
        """
        # Update target pose
        self.target_pose = msg.pose.pose

        # Compute target and current point/orientation
        target_point = (
            self.target_pose.position.x,
            self.target_pose.position.y,
            self.target_pose.position.z
        )
        curr_point = (
            self.curr_pose.position.x,
            self.curr_pose.position.y,
            self.curr_pose.position.z
        )
        target_yaw = quaternion_to_angle(self.target_pose.orientation)
        curr_yaw = quaternion_to_angle(self.curr_pose.orientation)

        # translation is target point - current point
        self.translation = tuple(map(lambda i, j: i - j, target_point, curr_point))
        
        # rotation is target yaw - current yaw
        self.rotation = tf.transformations.quaternion_from_euler(0, 0, target_yaw - curr_yaw)

    def send_camera_tf(self, msg):
        """
          Updates the camera pose and then publishes the transformation
            msg: nav_msgs/Odometry
        """
        # Update current camera pose
        self.curr_pose = msg.pose.pose

        # TODO: Might change to send at lower Hz since pose topic at 200Hz
        # Send Transform
        self.cam_br.sendTransform(
            self.translation,
            self.rotation,
            rospy.Time.now(),
            self.cam_frame,
            self.map_frame)

if __name__ == '__main__':
    rospy.init_node('camera_tf', anonymous=True)
    # Setup Params
    # odom_topic = rospy.get_param("/camera/odom/sample")                                            
    cam_odom_topic = "/camera/odom/sample"
    cam_frame = "/camera_odom_frame"
    map_frame = "/map"
    try:
        camera_tf = CameraTf(cam_odom_topic, cam_frame, map_frame)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
