[![Build Status](https://dev.azure.com/prl-mushr/mushr_pf/_apis/build/status/prl-mushr.mushr_pf?branchName=master)](https://dev.azure.com/prl-mushr/mushr_pf/_build/latest?definitionId=4&branchName=master)

# mushr_pf: Particle Filter
Particle filter (pf) for localization using the laser scanner. The pf requires a map and laser scan.

**Authors:**
Lirui Wang
Joseph Shieh
Chi-Heng Hung
Nansong Yi

### Install
**Note:** If you are not using the default MuSHR image then you will need to install [rangelibc](https://github.com/kctess5/range_libc).

Clone repo:
`cd ~/catkin_ws/src/ && git clone git@github.com:prl-mushr/mushr_pf.git`

### Running the PF
For real car: `roslaunch mushr_pf real.launch`  
For sim: `roslaunch mushr_pf sim.launch`

See [this tutorial](https://mushr.io/tutorials/navigation/) for more information.

### API
Parameters can be changed in `config/params.yaml`
#### Publishers
Topic | Type | Description
------|------|------------
`/car/pf/inferred_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) | Particle filter pose estimate
`/car/pf/viz/particles` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Partilcle array. Good for debugging
`/car/pf/viz/laserpose` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Pose fo the laser

#### Subscribers
Topic | Type | Description
------|------|------------
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) | Map the robot is in
`/car/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Current laserscan
`/car/vesc/sensors/servo_position_command` | [std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) | Current steering angle
`/car/vesc/sensors/core` | [vesc_msgs/VescStateStamped](https://github.com/prl-mushr/vesc/blob/master/vesc_msgs/msg/VescStateStamped.msg)| Current speed
