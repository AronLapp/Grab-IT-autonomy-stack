# grab_it_gz

## Status

Model is complete up to flange (Gripper fingers are missing). Calibration is roughly done but needs some adjustments (especially joints 3 and 4)

![Gazebo sim screenshot](images/gz_sim_model_03_02_26.png)

## Build

```bash
colcon build --packages-select grab_it_gz_sim --symlink-install
source install/setup.bash
```

## Run

Shell 1: Run gz sim with the robot model

```bash
ros2 launch grab_it_gz launch_gz_sim.launch.py
```

Shell 2: Send any command to joints using ros2_control
example:

```bash
ros2 topic pub -1 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'],
  points: [
    {
      positions: [1.0, -0.5, 1.5, 1.5, 1.5],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```
