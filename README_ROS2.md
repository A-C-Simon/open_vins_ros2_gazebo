# OpenVINS — ROS 2 Only

This copy of [OpenVINS](https://github.com/rpng/open_vins) (v2.7 based) has been
stripped down to **ROS 2 only**. All ROS 1 code, build files, launch files,
scripts and CI workflows have been removed. It targets **ROS 2 Humble** on
Ubuntu 22.04.

## Packages

| Package   | Content                                              | Executables                                                      |
|-----------|------------------------------------------------------|------------------------------------------------------------------|
| `ov_core` | Core frontend, camera models, simulation utilities   | `test_webcam`, `test_profile`                                    |
| `ov_init` | Static / dynamic visual-inertial initialization      | `test_simulation`, `test_dynamic_init`, `test_dynamic_mle`       |
| `ov_msckf`| MSCKF filter, ROS 2 visualizer, simulator            | `run_subscribe_msckf`, `run_simulation`, `test_sim_meas`, `test_sim_repeat` |
| `ov_eval` | Trajectory / timing evaluation tools                 | `error_*`, `timing_*`, `plot_trajectories`, `format_converter`   |
| `ov_data` | Groundtruth / dataset helper files (data only)       | –                                                                |

## Dependencies

```bash
sudo apt install libeigen3-dev libopencv-dev libopencv-contrib-dev \
  libboost-all-dev libceres-dev
# plus a ROS 2 Humble desktop install (rclcpp, cv_bridge, image_transport, tf2, ...)
```

## Build

From the workspace root (this repo must sit at `<ws>/src/open_vins`):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ov_core ov_init ov_msckf ov_eval ov_data \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

Live VIO node (EuRoC config shown, see `config/` for `tum_vi`, `kaist`,
`rpng_aruco`, `rs_d455`, `rs_t265`, ...):

```bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
# useful args: config_path, verbosity, use_stereo, max_cameras, rviz_enable
```

Monte-Carlo simulation (run from the **workspace root**, the sim trajectory
path in the config is relative to it):

```bash
cd <ws>
./build/ov_msckf/run_simulation src/open_vins/config/rpng_sim/estimator_config.yaml
```

## What was removed (vs upstream)

- `Dockerfile_ros1_*`, `.github/workflows/build_ros1.yml` and `build.yml`
  (the ROS-free standalone build relied on the deleted `ROS1.cmake` fallback)
- `ov_*/cmake/ROS1.cmake`, `find_package(catkin ...)` branches in every
  `CMakeLists.txt` (now `find_package(ament_cmake REQUIRED)` + `ROS2.cmake`)
- `ROS_VERSION`-conditioned deps in every `package.xml`
- `ROS1Visualizer.{h,cpp}`, `ros1_serial_msckf.cpp`
- ROS 1 `.launch` files (`serial`, `simulation`, `subscribe`, `dynamic`,
  `mle`, `record`) — the ROS 2 entry point is
  `ov_msckf/launch/subscribe.launch.py`
- `ov_msckf/scripts/` (`roslaunch`/`rosbag` based `run_ros_*` and `run_sim_*`)
- `ov_eval/python/` (`rospy` based `pid_ros.py`, `pid_sys.py`)
- Pure-ROS 1 sources: `ov_core/src/test_tracking.cpp`,
  `ov_eval/src/{live_align_trajectory,pose_to_file}.cpp`,
  `ov_eval/src/utils/Recorder.h`
- All `#if ROS_AVAILABLE == 1` branches (`set_node_handler`, `ros::ok()`,
  `ros::Time`, `rosbag`, ...) — `ROS_AVAILABLE=2` is now unconditional

Historical docs under `docs/` may still mention ROS 1; they are untouched.
