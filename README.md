# ropod_experiment_executor

A ROS package for modelling and executing robot experiments. Each experiment is modelled as a state machine; the definition and execution of state machine reuses most of the functionalities in [`mas_execution_manager`](https://github.com/b-it-bots/mas_execution_manager).

## Requirements

- toml
- [ropod_com_mediator](https://git.ropod.org/ropod/communication/ropod_com_mediator)
- [remote_monitoring](https://git.ropod.org/ropod/execution-monitoring/remote-monitoring)

## Installation

```
pip3 install toml
```

## Usage

1. `roscore`
2. `roslaunch ropod_com_mediator com_mediator.launch`
3. Launch remote monitoring `python3 app.py`
4. `roslaunch ropod_experiment_executor experiment_executor.launch`
5. Open `localhost:5000` and go to `Remote Monitoring` tab in the side bar.
