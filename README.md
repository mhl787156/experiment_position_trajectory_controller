# Position Trajectory Controller

This project provides a simple position trajectory controller for ROS2, MAVROS and (currently) PX4

This package contains just the controller `position_trajectory_controller` and utilises the `simple_offboard_msgs` for backwards compatibility. 

## Usage and Running

The most straightforward method is using docker in conjunction with the docker-compose scripts in ProjectStarling or Murmuration

```
docker run -it --rm --net=host uobflightlabstarling/position_trajectory_controller:latest
```

This library can also be recursively cloned in your local ros2 workspace and built. 
```
cd ros_ws/src
git clone --recursive https://github.com/StarlingUAS/position_trajectory_controller.git 
cd ../..
colcon build --packages-select position_trajectory_controller
ros2 launch position_trajectory_controller position_trajectory_controller.launch.xml
```

A kubernetes daemonset is provided for use with the flight arena. 

## *position_trajectory_controller* Node
This node is intended to simplify the programming of autonomous drone flight (`OFFBOARD` flight mode). It allows the setting of desired flight tasks and automatically transforms coordinates between frames. It is a high level system for interacting with the flight controller.

It also intends to simplify the execution of simple trajectory following based tasks.

It uses the [**libInterpolate**](https://github.com/CD3/libInterpolate) library for interpolation between trajectory points.

The node advertises the following service:

- `submit_trajectory` (srv/SubmitTrajectory) - Submits a position, velocity, attitude or rate trajectory to be executed.

```
# Submitted Trajectory
trajectory_msgs/JointTrajectory trajectory
# Trajectory type (must be set to 'position')
string type
# Interpolation method from https://github.com/CD3/libInterpolate (i.e. linear, cubic or monotonic), defaults to cubic
string interpolation_method
# Frame of the trajectory, defaults to map frame
string frame_id
# Whether drone should auto_arm, defaults to false
bool auto_arm # NOT USED
# Whether drone should wait for mission start, defaults to false, can be set to false if doing velocity, attitude or rates control
bool do_not_wait_for_mission_start # NOT USED
# Whether drone should takeoff first (for velocity, attitude or rates control)
bool auto_takeoff # NOT USED
float32 takeoff_height # NOT USED
---
string message
bool success
```

### Trajectory msg format

A `trajectory` is comprised of a [JointTrajectory](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectory.html) containing a list of [JointTrajectoryPoints](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html).

In order to determine whether the incoming trajectory is one of position, velocity, attitude or rates, we parse the following format based on the `type` given in the SubmitTrajectory and whether the `JointTrajectoryPoint` msgs populate the `position` or `velocity` field.

| Control Type 	| `type`   	| `JointTrajectoryPoint` field 	| index 0    	| index 1   	| index 2  	| index 3           	| index 4             	|
|--------------	|----------	|------------------------------	|------------	|-----------	|----------	|-------------------	|---------------------	|
| position     	| position 	| position                     	| x          	| y         	| z        	| yaw (optional)    	| yaw rate (optional) 	|

The first `JointTrajectoryPoint` in `trajectory` is used to determine the type of the trajectory. The time to execute each trajectory point is encoded as `time_from_start` field of `JointTrajectoryPoint`.

> **Note**: An error will be returned if the `trajectory` contains no points.

> **Note**: The `position` field of `JointTrajectoryPoint` is checked first. If it is populated by at least 3 values, it will assume they are valid points, otherwise it will check the `velocity` field. If neither is populated an error will be returned.

### Execution of the position trajectory.

Unless `do_not_wait_for_mission_start` is set, the executor will pause until it has received a mission start signal on `\mission_start` topic (std_msgs/Empty.msg). This can be done manually on the command line or through the [Starling UI](https://github.com/mhl787156/starling_ui_dashly)

The vehicle will automatically arm itself, and switch into `OFFBOARD` mode. It will then proceed to navigate to the first trajectory point given.

> **Note**: if the first position has a z value of that is too close to ground level (i.e. < 0.2m ) the following will fail. If this is an issue, it is suggested to have a special initial trajectory point for safe takeoff, or to set `auto_takeoff` to false. However note that takeoff does take a large amount of time, and to include that into the trajectory times if `auto_takeoff` is set to false.

Once the takeoff is complete, the service will return to the caller. The trajectory controller will begin executing the trajectory using the interpolation method chosen.

**The trajectory can be cancelled by sending a message on `\mission_abort` or `\emergency_stop`**

Once the time elapsed matches the `time_from_start` of the final trajectory point, the vehicle will land whever it has gotten to.

> **Note**: If the `time_from_start` are too short, then it is possible for the drone to never reach any of the points. It is recommended to give the vehicle a little more time to travel from one point to another.
## License

This project is covered under the MIT License.