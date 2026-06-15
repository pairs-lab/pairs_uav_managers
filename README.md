# pairs_uav_managers

Core control-and-estimation managers of the PAIRS UAV autonomy stack. This package
runs the high-level loops that fly the multirotor: it fuses sensors into a state
estimate, runs the active controller/tracker, and arbitrates flight constraints,
gains, and coordinate frames. It is the layer that turns reference trajectories into
the commands handed to the hardware API.

## Contents

Nodelets (loadable via `nodelet`):
- `ControlManager` — runs the active tracker/controller and publishes control output.
- `EstimationManager` — fuses state and above-ground-level (AGL) estimators into the UAV state.
- `UavManager` — high-level flight orchestration (takeoff, landing, and similar routines).
- `ConstraintManager` — selects and applies motion constraints.
- `GainManager` — selects and applies controller gains.
- `TransformManager` / `TfManager` — maintain the TF tree and frame transforms.

Pluginlib:
- `NullTracker` — a no-op `pairs_uav_managers::Tracker` plugin used as the default/idle tracker.

Per-manager launch files are provided under `launch/`.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)
```bash
sudo apt install ros-noetic-pairs-uav-managers
```

## Usage
Launch an individual manager, for example:
```bash
roslaunch pairs_uav_managers control_manager.launch
roslaunch pairs_uav_managers estimation_manager.launch
roslaunch pairs_uav_managers uav_manager.launch
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_managers` package; the original
copyright is retained in [LICENSE](LICENSE).
