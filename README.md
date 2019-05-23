# ropod_experiment_executor

A ROS package for modelling and executing robot experiments.  Each experiment is 
modelled as a state machine. 

The definition and execution of state machine reuses most of the functionalities 
in [`mas_execution_manager`](https://github.com/b-it-bots/mas_execution_manager).

## Requirements

- toml
- [ropod_com_mediator](https://git.ropod.org/ropod/communication/ropod_com_mediator)
- [remote_monitoring](https://git.ropod.org/ropod/execution-monitoring/remote-monitoring)

For path planner:
- [docker-overpass-api](https://git.ropod.org/ropod/wm/docker-overpass-api)
- [ropod_wm_mediator](https://git.ropod.org/ropod/wm/ropod_wm_mediator)
- [osm_bridge](https://git.ropod.org/ropod/wm/osm_bridge)
- [osm_bridge_ros_wrapper](https://git.ropod.org/ropod/wm/osm_bridge_ros_wrapper)

## Installation

```
pip3 install toml
```
Install instructions for other repositories are available in their READMEs.

<!--
## Usage

1. `roscore`
2. `roslaunch ropod_com_mediator com_mediator.launch`
3. Launch remote monitoring `python3 app.py`
4. `roslaunch ropod_experiment_executor experiment_executor.launch`
5. Open `localhost:5000` and go to `Remote Monitoring` tab in the side bar.
6. If path planner test needs to be run, then following additional things has to be started
   1. docker container with correct building
   2. `roslaunch osm_bridge_ros_wrapper osm_bridge.launch`
   3. `roslaunch ropod_wm_mediator wm_mediator.launch`

If testing navigation test without a real ropod, execute `rosrun
ropod_experiment_executor fake_navigation`
-->
