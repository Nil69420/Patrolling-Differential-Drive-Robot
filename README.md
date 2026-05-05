# Patrolling Differential Drive Robot (ROS2 Foxy)

BehaviorTree.CPP based patrolling package for ROS2 Foxy. The package is designed for differential-drive robots and includes TIAGo simulation integration, Nav2 action-driven motion, waypoint patrol logic, recharge behavior, and optional lifecycle-controlled tracking.

## What This Package Provides

- BT plugins:
  - `Recharge`
  - `Patrol`
  - `GetWaypoint`
  - `Move`
  - `BatteryChecker`
  - `TrackObjects`
- Main executable:
  - `patrolling_main`
- Launch flows:
  - `patrolling_bt.launch.py` (BT-only)
  - `patrolling.launch.py` (wrapper that can start simulation first)

## Behavior and Navigation Contract

The navigation behavior is validated by automated tests covering:

- required waypoint sequence traversal,
- Nav2 move success path,
- Nav2 move failure path (aborted goal),
- lifecycle tracking node activation/deactivation,
- battery gating behavior,
- motion command publication from patrol behavior.

## Requirements

- Ubuntu 20.04
- ROS2 Foxy
- colcon
- rosdep

## Build

From workspace root (example: `~/stuff`):

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select bt_patrolling
source install/setup.bash
```

## Launch

BT only:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch bt_patrolling patrolling_bt.launch.py
```

BT + simulation wrapper (default behavior):

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch bt_patrolling patrolling.launch.py
```

Wrapper simulation order and backend selection:

- default (`simulation_backend:=ignition`): try `ros_ign_gazebo` first, then fall back to classic Gazebo (`tiago_gazebo`, then `gazebo_ros`),
- force classic-first behavior: `simulation_backend:=classic`,
- auto mode (same practical behavior as ignition-first fallback): `simulation_backend:=auto`.

Force Ignition simulation path in wrapper (if `ros_ign_gazebo` is installed):

```bash
ros2 launch bt_patrolling patrolling.launch.py simulation_backend:=ignition ignition_world:=empty.sdf
```

Force classic TIAGo simulation path in wrapper:

```bash
ros2 launch bt_patrolling patrolling.launch.py simulation_backend:=classic world_name:=small_office
```

Disable simulator and run only BT:

```bash
ros2 launch bt_patrolling patrolling.launch.py start_tiago_gazebo:=false
```

## If TIAGo Sources Are Under `src/clones`

When TIAGo and PAL packages exist as source repos under `src/clones`, build them first so package discovery works at runtime:

```bash
source /opt/ros/foxy/setup.bash
colcon build --base-paths src/clones --packages-up-to tiago_gazebo --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## Test

Run full package tests:

```bash
source /opt/ros/foxy/setup.bash
colcon test --packages-select bt_patrolling
colcon test-result --verbose
```

Run targeted navigation contract tests directly:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
NAV_BIN=$(find build -type f -name navigation_contract_test | head -n 1)
TRACK_BIN=$(find build -type f -name tracking_lifecycle_test | head -n 1)
$NAV_BIN --gtest_filter=navigation_contract.required_waypoints_are_visited
$NAV_BIN --gtest_filter=navigation_contract.move_btn_success:navigation_contract.move_btn_failure
$TRACK_BIN --gtest_filter=tracking_lifecycle.track_objects_btn_1:tracking_lifecycle.track_objects_btn_2:tracking_lifecycle.track_objects_btn_3
```

## CI/CD

This repository includes GitHub Actions workflows for:

- linting and static quality checks,
- security scanning,
- optimized release-profile build verification,
- navigation contract test execution,
- release packaging on version tags.

See `.github/workflows` for full pipeline definitions.

## Groot / ZMQ

If ZeroMQ is available, Groot publisher support is enabled in `patrolling_main`. If the ZMQ port is already occupied, the process logs a warning and continues without telemetry.

## License

Apache-2.0
