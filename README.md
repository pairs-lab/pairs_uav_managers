# pairs_uav_managers

Core control-and-estimation managers of the PAIRS UAV autonomy stack. This package
runs the high-level loops that fly the multirotor: it fuses sensors into a state
estimate, runs the active controller/tracker, and arbitrates flight constraints,
gains, coordinate frames, and the safety area. It is the layer that turns reference
trajectories into the commands handed to the hardware API.

## Contents

Composable nodes (`rclcpp_components`):
- `ControlManager` — runs the active tracker/controller and publishes control output.
- `EstimationManager` — fuses state and above-ground-level (AGL) estimators into the UAV state.
- `UavManager` — high-level flight orchestration (takeoff, landing, and similar routines).
- `ConstraintManager` — selects and applies motion constraints.
- `GainManager` — selects and applies controller gains.
- `TransformManager` — maintains the TF tree and frame transforms.
- `SafetyAreaManager` — enforces the configured flight safety area.

Other:
- `NullTracker` — a no-op `pairs_uav_managers::Tracker` pluginlib plugin used as the default/idle tracker.
- `events_container` — executable that hosts manager events.

Per-manager Python launch files are provided under `launch/`.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-pairs-uav-managers
```

## Usage
Launch an individual manager, for example:
```bash
ros2 launch pairs_uav_managers control_manager.launch.py
ros2 launch pairs_uav_managers estimation_manager.launch.py
ros2 launch pairs_uav_managers uav_manager.launch.py
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_managers` package; the original
copyright is retained in [LICENSE](LICENSE).
