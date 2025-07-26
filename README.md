# fastbot_waypoints

This ROS2 package implements an **action server** for navigating the Fastbot to pre-defined 2D waypoints. The goal is to robot reach the target position and orientation based on action goals in simulation (ROS2 Galactic/Gazebo).

The main goal of the package is unit test with `gtest` to verify that robot reaches the goal position and orientation(within a small tolerance)

## Running testing

Make sure your ROS2 workspace built, sourced and Fastbot is ready within Gazebo environment.

- Start the waypoint server:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

- In second terminal, run testing:

```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

**Expected result:**
```
...
build/fastbot_waypoints/Testing/20250726-2040/Test.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/fastbot_waypoints/test_results/fastbot_waypoints/fastbot_waypoints_test.gtest.xml: 2 tests, 0 errors, 0 failures, 0 skipped

Summary: 3 tests, 0 errors, 0 failures, 0 skipped
```


### Testing for failure case

Modify **test/waypoints_test_ros_as.cpp** script to change the goals to something unachievable. Example:
```
...
Line 39     goal1_.x = 10.;
Line 40     goal1_.y = 10.;
Line 41     goal2_.x = -10.;
Line 42     goal2_.y = -10.;
...
```

Build the ROS2 workspace and run test node again. Since the goal won't be reached in time, test will result in time out and fail.
```
build/fastbot_waypoints/Testing/20250726-2112/Test.xml: 1 test, 0 errors, 1 failure, 0 skipped
build/fastbot_waypoints/test_results/fastbot_waypoints/fastbot_waypoints_test.gtest.xml: 1 test, 1 error, 0 failures, 0 skipped

Summary: 2 tests, 1 error, 1 failure, 0 skipped
```
