# Notes on T265 Integration

Our lab was interested in integrating Intel's T265 Tracking camera into the MuSHR platform's localization stack, which currently uses a particle filter where particles are propagated forward from motor commands and an Ackermann kinematic motion model, and resampled/reweighted using a sensor model comparing LIDAR readings to a precomputed 2D occupancy grid.

**Authors:**

Stefan Layanto

## Findings

We found that the T265 is not designed to be integrated as an additional sensor into an existing stack, but rather as more of a standalone solution to mobile localization. This [blog post/case study](https://www.intelrealsense.com/visual-inertial-tracking-case-study/) from Intel's website describes the motivation and design of the T265, describing it as a standalone tracking solution, fusing input from multiple sensors, and offloading processing workload from the host system. The T265 is able to read in output from a limited selection of other sensors (such as wheel odometry data) and incorporate that into its onboard algorithms that compute pose.

When tested by itself, the odometry output of the T265 was found to be much farther from ground truth than the existing particle filter and precomputed map setup (in terms of visual correctness and loop closure). However, [Intel's documentation notes that the T265 requires wheel odometry input](https://github.com/IntelRealSense/realsense-ros#using-t265) which we believe to cause the inaccuracy. The MuSHR platform does not have a wheel sensor but it does output a wheel odometry topic generated from motor commands. Steps to take going forward which we have not yet taken would be to connect this odometry topic to the T265 in order to improve its localization output, as well as to compare all trials against ground truth.

### Potential Advantages of the T265

Thinking from first principles, the camera provides several potential improvements to the existing localization stack:
 - Loop Closure - the T265 builds an internal map of 3D visual features, which is used by internal algorithms to relocalize and enable loop closure via pose jumps.
 - Inertial Navigation - the T265 houses an internal 6 DoF IMU. This would cover many weaknesses of the current kinematic model reading motor commands since it would have much more accurate orientation sensing, and could detect edge cases such as if the car gets stuck and does not move even though its wheels may be spinning.
 - SLAM - the existing stack requires a precomputed map in order for the sensor model to work. The T265 allows for operation in unmapped environments.
 - 3D localization - the T265 is able to localize itself in 3 dimensional space. The existing stack localizes in a world modelled as a flat 2 dimensional plane.
 - Reduced load on compute - the T265 is designed with custom onboard hardware and algorithms to efficiently handle sensor fusion by itself, freeing up compute on MuSHR platform.

 ### Difficulties of Integration

However, due to the design intent and methods of the T265, we found that it was difficult to effectively integrate the T265 in a way that leverages its capabilities. We could not incorporate our existing LIDAR and kinematic model into the camera's algorithms because the camera provides no such access/support. And we could not incorporate many of the T265's capabilities into our existing stack because the camera provides no access to its internal feature map, nor any intermediate output within its localization algorithms. The T265 only accepts additional input from a limited selection of sensors (such as wheel odometry) and only provides output in the form of raw images + IMU data, and final processed pose + twist. Additionally, being a fairly new sensor, support for the T265 is still being actively developed. Support can be limited in the Intel Realsense SDK, and even more so with the ROS Realsense Wrapper.


## Summary of Current Changes

We first integrated the T265 odometry into the motion model through using the difference between consecutive camera odometry readings to propagate the particles forward in place of the current kinematic motion model. A 'camera motion model' was created and the particle filter was modified to take in different motion models based on a keyword argument.

With some tuning. this approach was able to achieve loop closure and fairly accurate localization. However, it was clearly less accurate than the existing stack and was observed to be inaccurate when the camera faced featureless corridors or walls.

We briefly explored (but did not build) the idea of a hybrid motion model which would use both the camera and kinematic motion models to propagate particles forward, using a weighted sum to fuse both outputs.

We also began work on a camera sensor model to use the camera in the particle correction step rather than the propagation step. This would have functioned similarly to the camera motion model, with differential odometry computing an expected particle location, adding noise, then updating the weights of particles based on their location relative to the expected particle location. But this approach was eventually desisted since it did not really leverage any of the fundamental advantages  of the T265 as outlined above.

### Files Modified

particle_filter.py(modified) - modified to take a launch param string that specifies the type of motion model to use.

camera_motion_model.py - motion model use to propagate particles based on differential odometry output of T265

camera_tf.py - publishes a static transform from camera to map for visualization of camera odometry output. Transform is adjusted using 2D pose estimates in rviz.

utils.py(modified) - additional util functions added to convert odometry into x,y,theta which is how particles of the particle filter are stored.

camera_sensor_model.py - not completed. originally intended to use differential odometry output of camera (or twist) to reweigh particles.

bag_record.launch - used to record a bag of all topics except the second fisheye camera output of the T265 to save space

cam_tf.launch - starts the static camera transform

mushr_cam.launch - starts the T265

particle_filter.launch(modified) - modified to also call cam_tf.launch

sim.launch(modified) - modified to also call mushr_cam.launch



## Comparison of T265 Localization to Existing Stack

### Strengths of T265 over Existing Stack

- External compute custom-designed by Intel. Faster and reduces load on MuSHR resources.
- Inertial Measurement Unit + VIO very powerful for orientation estimation
- Camera constantly builds a 3D landmark map using SLAM which enables loop closure, allowing output pose to ‘jump’ relative to recognized landmarks.
- Does not require a precomputed map to localize and does not require LIDAR.

### Strengths of Existing Stack over T265

- LIDAR + map allows very precise localization in most scenarios, more so than the T265's SLAM.
- Kinematic motion model would be more accurate/precise than the generalized one that the T265 presumably uses when it factors in wheel odometry into its SLAM.
- LIDAR is not as vulnerable to lighting conditions as the T265's VIO would be.

### Simple Method to Combine T265 with Current Stack

The simplest method to integrate the T265 would be to use its twist output to project the expected pose of the car in the next iteration (similar to propagating particles), and then instead use it as a corrective step to reweight particles in the next iteration. This would be almost identical to the 'camera_sensor_model' mentioned at the end of the 'Summary of Current Changes' section, except it would use twist rather than pose.

Pros: easy to implement, uses T265 Inertial Navigation and Visual Inertial Odometry, and still incorporates LIDAR correction. Could potentially solve some kinematic motion model weaknesses such as if the robot were driving on ice.

Cons: Does not leverage the algorithms intel uses to integrate twist into odometry, and most likely does not use the 3D landmark map built by the camera since pose jumps are not included in the twist output. Does not allow for SLAM.

### Potential Alternative Method

A potential alternative method to the one mentioned above would be to use the odometry output of T265 rather than twist (using differential position to project expected pose of the car). While this seems simple enough, it is kind of like using two map frames (LIDAR map and T265' SLAM-generated 3D landmark map) that are anchored to each other at the current inferred position of the car, such that the transform between the two maps are continually adjusted in order to anchor them at the current inferred position of the car. This would prevent the T265's internal map from diverging from the LIDAR map over time.

Pros: odometry output incorporates Intel's algorithms and camera's internal map, which the simple method would not leverage. 

Cons: Implications not thoroughly understood as of now, and if not implemented correctly (or if the T265 SLAM map accumulates significant error) then pose jumps could ruin the localization.

### Useful Resources
 - [Short post on Intel's website on the design and purpose of the T265](https://www.intelrealsense.com/visual-inertial-tracking-case-study/)
 - [T265 Documentation](https://github.com/IntelRealSense/realsense-ros)
 - [Blog post documenting usage of T265 on a wheeled robot.](https://msadowski.github.io/Realsense-T265-First-Impressions/)
 - [Intel's Camera SDK (realsense)](https://github.com/IntelRealSense/librealsense)
 - [ROS Wrapper for the realsense SDK](https://github.com/IntelRealSense/realsense-ros)
 