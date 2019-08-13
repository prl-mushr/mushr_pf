[![Build Status](https://dev.azure.com/prl-mushr/mushr_pf/_apis/build/status/prl-mushr.mushr_pf?branchName=master)](https://dev.azure.com/prl-mushr/mushr_pf/_build/latest?definitionId=4&branchName=master)

# mushr_pf: Particle Filter
Particle filter (pf) for localization using the laser scanner. The pf requires a map and laser scan.

**Authors:**  
Lirui Wang  
Joseph Shieh  
Chi-Heng Hung  
Nansong Yi  

### Install  
**Note:** If you are not using the default mushr image then you will need to install [rangelibc](https://github.com/kctess5/range_libc).

Clone repo  
`cd ~/catkin_ws && git clone git@github.com:prl-mushr/mushr_pf.git`

### API
Parameters can be changed in `config/params.yaml`
#### Publishers
Topic | Type | Description
------|------|------------
`/pf/inferred_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) | Particle filter pose estimate
`/pf/viz/particles` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Partilcle array. Good for debugging
`/pf/viz/laserpose` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Pose fo the laser
`/odom` | [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)| Odom of car estimate from particle filter

#### Subscribers
Topic | Type | Description
------|------|------------
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) | Map the robot is in
`/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Current laserscan
`/vesc/sensors/servo_position_command` | [std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) | Current steering angle
`/vesc/sensors/core` | [vesc_msgs/VescStateStamped](https://github.com/prl-mushr/vesc/blob/master/vesc_msgs/msg/VescStateStamped.msg)| Current speed
