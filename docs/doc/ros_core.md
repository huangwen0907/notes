
# ROS Navigation
The navigation stack assumes that the robot is configured in a particular manner in order to run. The diagram above shows an overview of this configuration. The white components are required components that are already implemented, the gray components are optional components that are already implemented, and the blue components must be created for each robot platform. The pre-requisites of the navigation stack, along with instructions on how to fulfil each requirement, are provided in the sections below.
![overview_tf_small](http://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf_small.png)

## amcl
amcl is a probabilistic localization system for a robot moving in 2D. It implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map.
amcl takes in a laser-based map, laser scans, and transform messages, and outputs pose estimates. On startup, amclinitializes its **particle filter** according to the parameters provided. Note that, because of the defaults, if no parameters are set, the initial filter state will be a moderately sized particle cloud centered about (0,0,0).
### algorithms
Many of the algorithms and their parameters are well-described in the  book **Probabilistic Robotics**, by Thrun, Burgard, and Fox. The user is  advised to check there for more detail. In particular, we use the  following algorithms from that book:  **sample_motion_model_odometry**,  **beam_range_finder_model**,  **likelihood_field_range_finder_model**,  **Augmented_MCL**, and  **KLD_Sampling_MCL**.
As currently implemented, this node works only with laser scans and  laser maps. It could be extended to work with other sensor data.

###  Subscribed Topics

scan ([sensor_msgs/LaserScan]
-   Laser scans.

tf ([tf/tfMessage]
-   Transforms.

initialpose ([geometry_msgs/PoseWithCovarianceStamped]
-   Mean and covariance with which to (re-)initialize the particle filter.

map ([nav_msgs/OccupancyGrid]
-   When the  use_map_topic  parameter is set, AMCL subscribes to this topic to retrieve the map used for laser-based localization.  **New in navigation 1.4.2.**

### Published Topics
amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
- Robot's estimated pose in the map, with covariance.

particlecloud (geometry_msgs/PoseArray)
- The set of pose estimates being maintained by the filter.

tf (tf/tfMessage)
- Publishes the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map.

### Transforms
**amcl  transforms incoming laser scans to the odometry frame** (~odom_frame_id). So there must exist a path through the  [tf](http://wiki.ros.org/tf) tree from the frame in which the laser scans are published to the odometry frame.

An implementation detail: on receipt of the first laser scan,  amcl  looks up the transform between the laser's frame and the base frame (~base_frame_id), and latches it forever. So  amcl  cannot handle a laser that moves with respect to the base.

The drawing below shows the difference between localization using odometry and  amcl. During operation  amcl  estimates the transformation of the base frame (~base_frame_id) in respect to the global frame (~global_frame_id) but it only publishes the transform between the global frame and the odometry frame (~odom_frame_id). Essentially, this transform accounts for the drift that occurs using Dead Reckoning. The published transforms are  [future dated](http://wiki.ros.org/tf/FAQ#Why_do_I_see_negative_average_delay_in_tf_monitor.3F).
![amcl_localization](http://wiki.ros.org/amcl?action=AttachFile&do=get&target=amcl_localization.png)

## base_local_planner
This package provides implementations of the Trajectory Rollout and Dynamic Window approaches to local robot navigation on a plane. Given a plan to follow and a costmap, the controller produces velocity commands to send to a mobile base. This package supports both holonomic and non-holonomic robots, any robot footprint that can be represented as a convex polygon or circle, and exposes its configuration as ROS parameters that can be set in a launch file. This package's ROS wrapper adheres to the BaseLocalPlanner interface specified in the  [nav_core](http://wiki.ros.org/nav_core)  package.
- Maintainer status: maintained
- Maintainer: David V. Lu!! <davidvlu AT gmail DOT com>, Michael Ferguson <mfergs7 AT gmail DOT com>, Aaron Hoy <ahoy AT fetchrobotics DOT com>
- Author: Eitan Marder-Eppstein, Eric Perko, contradict@gmail.com
- License: BSD
- Source: git https://github.com/ros-planning/navigation.git (branch: kinetic-devel)

### Overview
The base_local_planner package provides a controller that drives a mobile base in the plane. This controller serves to connect the path planner to the robot. Using a map, the planner creates a kinematic trajectory for the robot to get from a start to a goal location. Along the way, the planner creates, at least locally around the robot, a value function, represented as a grid map. This value function encodes the costs of traversing through the grid cells. The controller's job is to use this value function to determine dx,dy,dtheta velocities to send to the robot.
![local_plan](http://wiki.ros.org/base_local_planner?action=AttachFile&do=get&target=local_plan.png)
The basic idea of both the Trajectory Rollout and Dynamic Window Approach (DWA) algorithms is as follows:

- 1.  Discretely sample in the robot's control space (dx,dy,dtheta)
- 2.  For each sampled velocity, perform forward simulation from the robot's current state to predict what would happen if the sampled velocity were applied for some (short) period of time.
- 3.  Evaluate (score) each trajectory resulting from the forward simulation, using a metric that incorporates characteristics such as: proximity to obstacles, proximity to the goal, proximity to the global path, and speed. Discard illegal trajectories (those that collide with obstacles).
- 4.  Pick the highest-scoring trajectory and send the associated velocity to the mobile base.
- 5.  Rinse and repeat.

DWA differs from Trajectory Rollout in how the robot's control space is sampled. Trajectory Rollout samples from the set of achievable velocities over the entire forward simulation period given the acceleration limits of the robot, while DWA samples from the set of achievable velocities for just one simulation step given the acceleration limits of the robot. This means that DWA is a more efficient algorithm because it samples a smaller space, but may be outperformed by Trajectory Rollout for robots with low acceleration limits because DWA does not forward simulate constant accelerations. In practice however, we find DWA and Trajectory Rollout to perform comparably in all our tests and recommend use of DWA for its efficiency gains.

**Useful references:**
-   [Brian P. Gerkey and Kurt Konolige. "Planning and Control in Unstructured Terrain "](https://pdfs.semanticscholar.org/dabd/bb636f02d3cff3d546bd1bdae96a058ba4bc.pdf?_ga=2.75374935.412017123.1520536154-80785446.1520536154). Discussion of the Trajectory Rollout algorithm in use on the LAGR robot.
    
-   [D. Fox, W. Burgard, and S. Thrun. "The dynamic window approach to collision avoidance".](http://www.cs.washington.edu/ai/Mobile_Robotics/postscripts/colli-ieee.ps.gz)  The Dynamic Window Approach to local control.
    
-   [Alonzo Kelly. "An Intelligent Predictive Controller for Autonomous Vehicles"](http://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_7/kelly_alonzo_1994_7.pdf). An previous system that takes a similar approach to control.
## global_planner
This package provides an implementation of a fast, interpolated global planner for navigation. This class adheres to the nav_core::BaseGlobalPlanner interface specified in the [nav_core](http://wiki.ros.org/nav_core) package. It was built as a more flexible replacement to [navfn](http://wiki.ros.org/navfn), which in turn is based on [NF1](http://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf).
###  Examples of Different Parameterizations
**Standard Behavior**
![](http://wiki.ros.org/global_planner?action=AttachFile&do=get&target=GlobalPlanner.png)

### Map Grid

In order to score trajectories efficiently, the Map Grid is used. For each control cycle, a grid is created around the robot (the size of the local costmap), and the global path is mapped onto this area. This means certain of the grid cells will be marked with distance 0 to a path point, and distance 0 to the goal. A propagation algorithm then efficiently marks all other cells with their manhattan distance to the closest of the points marked with zero.

This map grid is then used in the scoring of trajectories.

The goal of the global path may often lie outside the small area covered by the map_grid, when scoring trajectories for proximity to goal, what is considered is the "local goal", meaning the first path point which is inside the area having a consecutive point outside the area. The size of the area is determined by move_base.

###  Oscillation Suppression
Oscillation occur when in either of the x, y, or theta dimensions, positive and negative values are chosen consecutively. To prevent oscillations, when the robot moves in any direction, for the next cycles the opposite direction is marked invalid, until the robot has moved beyond a certain distance from the position where the flag was set.

###  Generic Local Planning
The groovy release of ROS includes a new implementation of the  [dwa_local_planner](http://wiki.ros.org/dwa_local_planner)  package. The implementation attempts to be more modular, to allow easier creation of custom local planners while reusing a lot of code. The code base of base_local_planner has been extended with several new headers and classes.

The principle of local planning is the search for a suitable local plan in every control cycle. For that purpose, a number of candidate trajectories is generated. For a generated trajectory, it is checked whether it collides with an obstacle. If not, a rating is given to compare several trajectories picking the best.

Obviously, depending on the robot shape (and actuator shape) and the domain, this principle can be instantiated in very different ways. There are many special ways to generate trajectories, and ways to search the space of potential trajectories for a most suitable one.

The interfaces and classes below capture the generic local planning principles allowing many instantiations. It should be possible to create custom local planners using the  dwa_local_planner  as template and just adding own cost functions or trajectory generators.
#### TrajectorySampleGenerator
This interface describes a Generator which may generate a finite or infinte number of trajectories, returning a new one on each invocation of  nextTrajectory().
The  SimpleTrajectoryGenerator  class can generate trajectories described in the overview, using either the trajectory rollout or the DWA principle.

#### Trajectory  Cost Function

This interface contains most importantly  scoreTrajectory(Trajectory &traj), which takes a trajectory and returns a score. A negative score means the trajectory is invalid. For positive value, the semantics are that a trajectory with a lower score is preferrable to one with a higher score with respect to this cost function.

Each cost function also has a scale by which its impact can be altered in comparison to the other cost functions.

The  base_local_planner  package ships with some cost functions used on the PR2, described below.

#### SimpleScoredSamplingPlanner

This is a simple implementation of a trajectory search, taking a  TrajectorySampleGenerator  and a list of  TrajectoryCostFunction. It will invoke  **nextTrajectory()**  until the generator stops generating trajectories. For each, trajectory, it will loop over the list of cost functions, adding up their positive values or aborting the scoring if one cost function returns a negative value.

Using the scale of the cost functions the result is then the Trajectory with the best weighted sum of positive cost function results.
### TrajectoryPlannerROS

The  base_local_planner::TrajectoryPlannerROS  object is a  [wrapper](http://wiki.ros.org/navigation/ROS_Wrappers)  for a  base_local_planner::TrajectoryPlanner  object that exposes its functionality as a  [C++ ROS Wrapper](http://wiki.ros.org/navigation/ROS_Wrappers). It operates within a ROS namespace (assumed to be  _name_  from here on) specified on initialization. It adheres to the  nav_core::BaseLocalPlanner  interface found in the  [nav_core](http://wiki.ros.org/nav_core)  package.

Example creation of a  base_local_planner::TrajectoryPlannerROS  object:
```
   1 #include <tf/transform_listener.h>
   2 #include <costmap_2d/costmap_2d_ros.h>
   3 #include <base_local_planner/trajectory_planner_ros.h>
   4 
   5 ...
   6 
   7 tf::TransformListener tf(ros::Duration(10));
   8 costmap_2d::Costmap2DROS costmap("my_costmap", tf);
   9 
  10 base_local_planner::TrajectoryPlannerROS tp;
  11 tp.initialize("my_trajectory_planner", &tf, &costmap);
```

### costmap_2d
This package provides an implementation of a 2D costmap that takes in sensor data from the world, builds a 2D or 3D occupancy grid of the data (depending on whether a voxel based implementation is used), and inflates costs in a 2D costmap based on the occupancy grid and a user specified inflation radius. This package also provides support for map_server based initialization of a costmap, rolling window based costmaps, and parameter based subscription to and configuration of sensor topics.

## move_base
The  move_base  node provides a ROS interface for configuring, running, and interacting with the  [navigation stack](http://wiki.ros.org/navigation)  on a robot. A high-level view of the move_base node and its interaction with other components is shown above. The blue vary based on the robot platform, the gray are optional but are provided for all systems, and the white nodes are required but also provided for all systems. For more information on configuration of the  move_base  node, and the navigation stack as a whole, please see the  [navigation setup and configuration](http://wiki.ros.org/navigation/Tutorials/RobotSetup)  tutorial.
![overview_tf_small](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=overview_tf_small.png)

**Expected Robot Behavior**

Running the  move_base  node on a robot that is properly configured (please see  [navigation stack documentation](http://wiki.ros.org/navigation)  for more details) results in a robot that will attempt to achieve a goal pose with its base to within a user-specified tolerance. In the absence of dynamic obstacles, the move_base node will eventually get within this tolerance of its goal or signal failure to the user. The  move_base  node may optionally perform recovery behaviors when the robot perceives itself as stuck. By default, the  [move_base](http://wiki.ros.org/move_base)  node will take the following actions to attempt to clear out space:

First, obstacles outside of a user-specified region will be cleared from the robot's map. Next, if possible, the robot will perform an in-place rotation to clear out space. If this too fails, the robot will more aggressively clear its map, removing all obstacles outside of the rectangular region in which it can rotate in place. This will be followed by another in-place rotation. If all this fails, the robot will consider its goal infeasible and notify the user that it has aborted. These recovery behaviors can be configured using the  [recovery_behaviors](http://wiki.ros.org/move_base#Parameters)  parameter, and disabled using the  [recovery_behavior_enabled](http://wiki.ros.org/move_base#Parameters)  parameter.
![](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=recovery_behaviors.png)

### nav_core
This package provides common interfaces for navigation specific robot actions. Currently, this package provides the BaseGlobalPlanner, BaseLocalPlanner, and RecoveryBehavior interfaces, which can be used to build actions that can easily swap their planner, local controller, or recovery behavior for new versions adhering to the same interface.
**Overview**
![move_base_interfaces](http://wiki.ros.org/nav_core?action=AttachFile&do=get&target=move_base_interfaces.png)

## robot_pose_ekf

The Robot Pose EKF package is used to estimate the 3D pose of a robot, based on (partial) pose measurements coming from different sources. It uses an extended Kalman filter with a 6D model (3D position and 3D orientation) to combine measurements from wheel odometry, IMU sensor and visual odometry. The basic idea is to offer loosely coupled integration with different sensors, where sensor signals are received as ROS messages.
####  How to use the Robot Pose EKF
**Configuration**
A default launch file for the EKF node can be found in the robot_pose_ekf package directory. The launch file contains a number of configurable parameters:

-   freq: the update and publishing frequency of the filter. Note that a higher frequency will give you more robot poses over time, but it will not increase the accuracy of each estimated robot pose.
-   sensor_timeout: when a sensor stops sending information to the filter, how long should the filter wait before moving on without that sensor.
-   odom_used, imu_used, vo_used: enable or disable inputs.

The configuration can be modified in the launch file, which looks something like this:

     <launch>
      <node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
        <param name="output_frame" value="odom"/>
        <param name="freq" value="30.0"/>
        <param name="sensor_timeout" value="1.0"/>
        <param name="odom_used" value="true"/>
        <param name="imu_used" value="true"/>
        <param name="vo_used" value="true"/>
        <param name="debug" value="false"/>
        <param name="self_diagnose" value="false"/>
      </node>
     </launch>

### How Robot Pose EKF works

#### Pose interpretation
All the sensor sources that send information to the filter node can have their own  _world_  reference frame, and each of these  _world_  reference frames can drift arbitrary over time. Therefore, the  **absolute poses**  sent by the different sensors cannot be compared to each other. The node uses the  **relative pose differences**  of each sensor to update the extended Kalman filter.
#### Covariance interpretation
As a robot moves around, the uncertainty on its pose in a world reference continues to grow larger and larger. Over time, the covariance would grow without bounds. Therefore it is not useful to publish the covariance on the pose itself, instead the sensor sources publish how the covariance changes over time, i.e. the covariance on the velocity.  _Note that using observations of the world (e.g. measuring the distance to a known wall) will reduce the uncertainty on the robot pose; this however is localization, not odometry._
#### Timing
Imagine the robot pose filter was last updated at time t_0. The node will not update the robot pose filter until at least one measurement of  _each_  sensor arrived with a timestamp later than t_0. When e.g. a message was received on the  odom  topic with timestamp $t_1 > t_0$, and on the  imu_data  topic with timestamp $t_2 > t_1 > t_0$, the filter will now update to the latest time at which information about all sensors is available, in this case to time t_1. The odom pose at $t_1$ is directly given, and the imu pose at $t_1$ is obtained by linear interpolation of the imu pose between $t_0$ and $t_2$. The robot pose filter is updated with the relative poses of the odom and imu, between $t_0$ and $t_1$.
![robot_pose_ekf](http://wiki.ros.org/robot_pose_ekf?action=AttachFile&do=get&target=robot_pose_ekf.png)
The above figure shows experimental results when the PR2 robot started from a given initial position (green dot), driven around, and returned to the initial position. A perfect odometry x-y plot should show an exact loop closure. The blue line shows the input from the wheel odometry, with the blue dot the estimated end position. The red line shows the output of the  robot_pose_ekf, which combined information of wheel odometry and imu, with the red dot the estimated end position.