# Autonomy
Code for Moon Ranger's autonomy computer lives here (in a ROS workspace)

The autonomy repository has the following components:
- System Executive
  - Decides what to do when. This components hosts the state machine which makes all the decisions.
- Global Planner
  - Keeps information in the global sense like the global map and generates plan on them. For example, given a goal generates a path which avoids craters and shadows.
- Navigator
  - Operates on the local frame, issues drive commands and navigates the robot avoiding non-traversable regions as the come up.
- Position Estimator
  - Estimates the position of the robot in both the local frame to be used by the navigator and global frame to reference the global map.
- Wifi Gateway
  - Main entry point to the autonomy computer. Talks to the wifi chip and transmits information back and forth including images and telemetry.
- Intercom
  - Similar to the wifi-gateway but talks to the embedded computer. Gets information like IMU and encoders and sends motor commands, state updates etc. 
- Health Monitor
  - Monitors all the faults and reports to the executive. Generally does not take any actions execept restarting the executive when it is not active.
- Telemetry Manager 
  - Responsible for packaging and sending telemetry information to the embedded computer and back to ground.
- Camera Driver
  - Controls the cameras and receives images from them.
- autonomy_common_msgs
  - ROS messages which are common between the different nodes live here. This is to make sure we do not run into circular dependencies.

## Setup
- Clone the workspace using the clone command
`git clone https://github.com/PlanetaryRobotics/autonomy.git`

- This should create the `autonomy` ROS workspace. To build, go to this directory and use `catkin_make`
```
cd autonomy
catkin_make
```

- To build the unit-tests use `catkin_make` with the following option
```
catkin_make --make-args tests
```

- To run the unit-tests use
```
catkin_make --make-args run_tests
```


