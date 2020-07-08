# Notes on T265 Integration

Intel's T265 Tracking camera was added to the Mushr platform, so the lab was interested in exploring ways to integrate it into the existing localization stack, which uses a particle filter where particles are propagated with an Ackermann kinematic motion model reading motor commands, and resampled using a sensor model comparing LIDAR readings to a precomputed 2D occupancy grid.

**Authors:**

Stefan Layanto

## Findings

We found that the T265 is not designed to be integrated as an additional sensor into an existing stack. It is designed to be a standalone solution to localization. In its current  state, the odometry output of the camera was not very accurate because we have not yet given the camera wheel odometry as an input, which it needs for robust localizationon wheeled robots. While there are ways of adding it into the sensor model of our current stack, it would be hard to do this in a way that fully leverages many of the camera’s capabilities such as the loop closure achieved from localization within its internal map.

## Summary of Current Changes

We first integrated the T265 odometry into the motion model, and used the difference between consecutive camera odometry readings to propagate the particles forward. A camera motion model was created and the particle filter was modified to take in a motion model for a keyword argument. We also explored the option of a hybrid motion model, but did not begin building it. We also began work on a camera sensor model to use the camera in the particle correction step rather than the propagation step. This would have functioned similarly to the camera motion model, with differential odometry computing an expected particle location, adding noise, then updating the weights of particles based on their location.

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

### Strengths of T265 over existing stack

- External compute custom-designed by Intel. Faster and reduces load on MuSHR resources.
- Inertial Measurement Unit + VIO very powerful for orientation estimation
- Camera constantly builds a 3D landmark map using SLAM which enables loop closure, allowing output pose to ‘jump’ relative to recognized landmarks.
- Does not require a precomputed map to localize and does not require LIDAR.

### Strengths of existing stack oover T265

- LIDAR + map allows very precise localization in most scenarios, more so than the T265's SLAM.
- Kinematic motion model would be more accurate/precise than the generalized one that the T265 presumably uses when it factors in wheel odometry into its SLAM.
- LIDAR is not as vulnerable to lighting conditions as the T265's VIO would be.

### Simplest way to combine T265 with Current Stack

Use twist output of T265 to project the expected pose of the car (similar to propagating particles) but then instead use it as a corrective step to reweight particles.

Pros: easy to implement, uses T265 IMU and VIO, still get LIDAR correction. Could potentially solve the problem of the robot driving on ice.

Cons: Does not use whatever algorithms intel uses to integrate twist into odometry, and most likely does not use the 3D landmark map built by the camera. Effectively, we wouldn’t be using the algorithms intel uses for loop closure/pose jumps

### Suggestions:

Use odometry output of T265  rather than twist. Use differential position to project expected pose of the car.

Pros: using odometry output would use the features of the camera that the simple approach would not use. It is kind of like using two map frames (lidar map and T265 3d landmarks from SLAM) that are anchored to each other at the inferred position of the car, such that as the T265 SLAM map diverges due to error, the relative transform between the T265 map and the lidar map would be anchored at where they both think is the car’s current position. Hard to explain with words.

Cons: Implications not thoroughly understood as of now, and if not implemented correctly (or if the T265 SLAM map is way off) then pose jumps could ruin the localization.
