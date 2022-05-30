# Synchronous Position Trajectory Controller

This project provides a synchonous simple position trajectory controller for ROS2, MAVROS and (currently) PX4 for multi-vehicle flight

This package is an extension of the `position_trajectory_controller` (https://github.com/StarlingUAS/position_trajectory_controller) with the new functionality

This package contains the controller `position_trajectory_controller`, a syncrhonisation monitor `sync_monitor`, some `synchronous_msgs` and utilises the `simple_offboard_msgs` from [`simple_offboard`](https://github.com/StarlingUAS/starling_simple_offboard) for backwards compatibility with the [`starling-allocator`](https://github.com/StarlingUAS/starling_allocator) and [`starling-ui-dashly`](https://github.com/StarlingUAS/starling_ui_dashly).

## Installation
### Pre-requisits
This project uses Starling as its base. Therefore you will need thos prerequisites, please see [Starling documentation](https://docs.starlinguas.dev). This is primarily docker. You do not need to clone this project.

## Basic Usage

### Run the docker container
The most straightforward method is using docker in conjunction with the docker-compose scripts in ProjectStarling or Murmuration

```
docker run -it --rm --net=host uobflightlabstarling/position_trajectory_controller:latest
```

### Building the docker container locally
First you will need to recursively clone the repo into your workspace, cd into the directory and then run `make`:
```
git clone --recursive https://github.com/StarlingUAS/synchronous_position_trajectory_controller.git
cd synchronous_position_trajectory_controller
make
```

### Building Locally
This library can also be recursively cloned in your local ros2 workspace and built.
```
cd ros_ws/src
git clone --recursive https://github.com/StarlingUAS/position_trajectory_controller.git
cd ../..
colcon build --packages-select position_trajectory_controller
ros2 launch position_trajectory_controller position_trajectory_controller.launch.xml
```

A kubernetes daemonset is provided for use with the flight arena.

## Running in Simulation

### Starling CLI
his project then uses the [Murmuration project](https://github.com/StarlingUAS/Murmuration) in order to simplify the multi-vehicle control and simulation. Murmuration includes a Command Line interface and example scripts to help run multi-vehicle simulations. In particular the structure of these simulations directly matches the real life deployment within the flight arena.

You will need to clone the Murmuration project somewhere into your workspace, recommendation is in the parent directory to this one.
```
git clone https://github.com/StarlingUAS/Murmuration.git
```

Following the instructions from Murmuration, it is recommended you add the Murmuration file path to your PATH. Either run in each terminal:
```
export PATH="$(pwd)":$PATH
```

or add the following to the bottom of your bashrc/zshrc file:
```
export PATH="<path to Marsupial root folder>":$PATH
```

then with the Starling CLI on the path, you should be able to run
```
starling install
```

### Running the simulation

First start the simulated cluster (here we start an example with 2 vehicles)
```
starling start kind -n 2
starling simulator start --load
```

After locally building the container (running `make`) you will need to upload it to the simulator
```
make
starling utils kind-load uobflightlabstarling/position-trajectory-controller:latest
```

Optionally, you can also load in the allocator and the dashboard
```
starling utils kind-load uobflightlabstarling/starling-allocator:latest
starling utils kind-load mickeyli789/starling-ui-dashly:latest
```

Finally, you can start this simulation by running
```
starling deploy -f kubernetes
```

To stop or restart the simulation, you can run:
```
starling deploy -f kubernetes stop
starling deploy -f kubernetes restart
```

Then you can open a browser page at https://localhost:8080 for gazebo and https://localhost:3000 for the user interface page.

You may also want to start the dashboard using the following command which opens on https://localhost:31771
```
starling start dashboard
```

### During the simulation

To get the vehicle to follow a path plan, first go to the user interface webpage and click on `Load Trajectories` in the top right hand corner. This will bring you to a page where you can load a particular trajectory for your vehicles to follow. This page allows you to load single or multiple trajectories (see `help` button for details.)

In the trajectories directory of this repository, there are an number of sample trajectories. You can drag and drop one of the json files within these directories into the select file box of the webpage. This will load up that trajectory, and show it on the right handside.

Clicking confirm & submit will bring up a verifying box. For now please select manual allocation method, and then select which vehicle to allocate to which trajectory. Once happy, please submit and return to control panel. This will submit the trajectories to the allocator, which will then allocate the trajectories directly to the vehicles.

The vehicles will then do the following:

1. Initialise and wait for the Mission Go signal to continue to takeoff
2. Takeoff to 0.5m above its current location, then wait for Mission Go signal to continue to the start of the trajectory
3. Vehicle will then fly to the start of the trajectory (may jerk there if it is far away from current location), then wait for Mission Go signal to then start executing the rest of the trajectory in sync.
4. Vehicle will then automatically land when its trajectory is finished.


## Implementation

### *position_trajectory_controller* Node

This node is intended to simplify the programming of autonomous drone flight (`OFFBOARD` flight mode). It allows the setting of desired flight tasks and automatically transforms coordinates between frames. It is a high level system for interacting with the flight controller.

With multiple drones, it also attempts to synchronise flight if vehicles start falling behind.

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

#### Trajectory msg format

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

### Synchronisation

It has been noticed that pure open loop path following by multi-drones often leads to drones flying into or close to each other due to lag in following given trajectories.

Therefore we devise a simple scheme to keep drones relatively in sync by delaying other vehicles when they complete a task.

1. The vehicle is detected to arrive at its current assigned waypoint.
2. If a vehicle detects it is late (with respect to the orignal plan), it will send a delay message to the synchronisation monitor. The sync monitor will forward this delay to be stored by all other vehicles.
3. Then the vehicle calculates its 'instantaneous delay' or the amount of time it needs to wait to even out the service. This is defined as the maximum over all vehicles delays minus this vehicles current delay.
4. The vehicle then stays at the same waypoint until the calculated time of the vehicles cumulative delays, plus the vehicles instantaneous delay.
5. If the delay is completed (i.e. exceeded the delay's wait until time), the vehicle contiues to its next waypoint.

The `synchronous_msgs` package contains the following messages on topics to facilitate this synchronisation.

- On a vehicle's delay, it will send a `NotifyDelay` msg to the `/monitor/notify_delay` topic, containing the vehicle name, delay, expected arrival time and actual arrival time.
- On receipt of a delay, the monitor will then identify live vehicles from broadcast ROS topics, and send a `NotifyPause` message to each vehicles `/<vehicl_id>/notify_pause` topic, containing the delay and the vehicle name causing it.

## License

This project is covered under the MIT License.