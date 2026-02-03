# grab_it_gz

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
  joint_names: ['joint_1', 'joint_2'],
  points: [
    {
      positions: [1.0, -0.5],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```
