# Lab 1 – ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems — Alexandria National University, Faculty of Engineering.

## Team

youssef ahmed [--]
mohamed hassan [--]
abdelrhman hany[--]

## Packages

| Package | Type | Contents |
|---------|------|----------|
| `turtle_interfaces` | ament_cmake | Custom message `TargetPose`, actions `GoToPose` and `DrawSquare`, service `SetSpeedLimits` |
| `turtle_controller` | ament_python | Nodes `go_to_pose_server`, `reset_server`, `square_server` |
| `turtle_bringup` | ament_python | Launch files, YAML parameters, recorded bag file |

### Interfaces (`turtle_interfaces`)

```
msg/TargetPose.msg         float64 x, y, yaw
action/GoToPose.action     goal:     TargetPose target
                           result:   success, final_pose, position_error, yaw_error, message
                           feedback: current_pose, remaining_distance, remaining_angle, phase
action/DrawSquare.action   goal:     float64 side_length   (<= 0 uses the default)
                           result:   success, sides_completed, message
                           feedback: current_side, current_target, remaining_distance
srv/SetSpeedLimits.srv     request:  max_linear_speed, max_angular_speed
                           response: success, message
```

### Nodes (`turtle_controller`)

| Node | Provides | Uses |
|------|----------|------|
| `go_to_pose_server` | action `/go_to_pose`, service `/set_speed_limits`, parameters `max_linear_speed`, `max_angular_speed` (+ gains/tolerances) | `/turtle1/pose`, `/turtle1/cmd_vel` |
| `reset_server` | service `/reset_turtle` (`std_srvs/Trigger`) | turtlesim `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/clear` |
| `square_server` | action `/draw_square` | `/reset_turtle`, `/go_to_pose` (4 goals), `/turtle1/pose` |

**goToPose:** turn to face the target, drive with a proportional controller (heading corrected
continuously, angle errors wrapped to [-pi, pi)), then turn to the final yaw. The speed limits are
re-read on every control cycle (50 Hz), so changing them affects a turtle that is already moving.
While a goal is running, new goals are rejected, and a cancel stops the turtle.

**Reset:** lifts the pen, teleports to (5.544, 5.544, 0), clears the screen, then puts the pen down
again. Because the pen is up while teleporting, no line is drawn to the start.

**Square:** calls reset, then sends 4 goToPose goals (one per corner, counter-clockwise from the
start) and waits for each to finish. Each corner's yaw points along the next side, so every side
starts already aligned. Squares that would leave [0.5, 10.5] are rejected.

## Build

Tested on Ubuntu 26.04 with ROS 2 Lyrical.

```bash
sudo apt install ros-lyrical-turtlesim          # if not installed yet
mkdir -p ~/lab1_ws/src && cd ~/lab1_ws/src
git clone https://github.com/youssefalkady/design-lab.git
cd ~/lab1_ws
source /opt/ros/lyrical/setup.bash
colcon build
source install/setup.bash                       # in every new terminal
```

## Run

```bash
# Whole system: turtlesim + all servers + parameters from config/turtle_params.yaml
ros2 launch turtle_bringup turtle_system.launch.py

# Override the speed limits from the launch command
ros2 launch turtle_bringup turtle_system.launch.py max_linear_speed:=0.5 max_angular_speed:=1.0

# Start the system and draw a square automatically
ros2 launch turtle_bringup draw_square.launch.py side_length:=3.0
```

Other parameters (gains, tolerances, start pose, default side length) are in
[turtle_bringup/config/turtle_params.yaml](turtle_bringup/config/turtle_params.yaml); you can
pass another file with `params_file:=/path/to/file.yaml`.

## Testing each requirement

Run these in a second terminal (sourced) while `turtle_system.launch.py` is running.

**R1 / R6 — turtlesim and launch files**
```bash
ros2 node list        # /turtlesim /go_to_pose_server /reset_server /square_server
ros2 topic list       # /turtle1/pose /turtle1/cmd_vel ...
ros2 service list     # /clear /reset /spawn /turtle1/teleport_absolute ... /reset_turtle /set_speed_limits
ros2 action list      # /go_to_pose /draw_square
```

**R2 — interfaces**
```bash
ros2 interface package turtle_interfaces
ros2 interface show turtle_interfaces/msg/TargetPose
ros2 interface show turtle_interfaces/action/GoToPose
ros2 interface show turtle_interfaces/action/DrawSquare
ros2 interface show turtle_interfaces/srv/SetSpeedLimits
```

**R3 — goToPose action**
```bash
ros2 action send_goal --feedback /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 8.5, y: 9.0, yaw: 3.14}}"
ros2 action send_goal --feedback /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 2.0, y: 2.0, yaw: -1.57}}"
ros2 topic echo --once /turtle1/pose          # compare with the goal
```
The result reports `position_error` and `yaw_error`. In our tests they were below 0.05 m and
0.01 rad for goals in every direction.

**R7 — speed parameters and service**
```bash
ros2 param list /go_to_pose_server
ros2 param get /go_to_pose_server max_linear_speed
ros2 param set /go_to_pose_server max_linear_speed 1.0        # use a decimal point (double)
ros2 param set /go_to_pose_server max_linear_speed -1.0       # refused: must be > 0

ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 0.5, max_angular_speed: 0.8}"
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: -2.0, max_angular_speed: 1.0}"   # success: false
```
To see a live change, send a long goal and call the service while the turtle is moving.

**R4 — reset service**
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 topic echo --once /turtle1/pose          # x: 5.544 y: 5.544 theta: 0.0
```

**R5 — square action**
```bash
ros2 action send_goal --feedback /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}"
```
The `go_to_pose_server` log shows exactly 4 goals (`goal #n`) per square. If you run it again, it
starts on a clean screen. A side of 6.0 is rejected because the square would leave the window.

**Unit tests** (controller logic, no ROS needed): `colcon test --packages-select turtle_controller && colcon test-result --verbose`

## Bonus: design patterns

- **State** — [motion_states.py](turtle_controller/turtle_controller/motion_states.py).
  goToPose has three phases (`RotateToTarget`, `DriveToTarget`, `RotateToYaw`) plus a `Reached`
  state. Each phase is a class with `step(ctx)`, which returns a velocity command and the next
  state. The action server (the context) only calls `state.step()` until `state.done`. This
  removes a large `if phase == ...` block, keeps each phase's logic and exit condition in one
  place, and lets the phases be unit-tested without ROS (see `test/test_motion_states.py`).
  We used this pattern because the controller's behaviour depends on which phase it is in.
- **Facade** — [turtlesim_facade.py](turtle_controller/turtle_controller/turtlesim_facade.py).
  Each turtlesim service call needs a client, a request object, an availability check, an async
  call and a wait with a timeout. `TurtlesimFacade` hides this behind `set_pen()`,
  `teleport_absolute()` and `clear()` and turns failures into a single `TurtlesimError`. The reset
  callback is then four readable lines, and the service names and types live in one place.
- **Observer** (provided by ROS) — the speed limits use the parameter callbacks
  (`add_on_set_parameters_callback` validates, `add_post_set_parameters_callback` logs). The same
  validation runs whether the change comes from `ros2 param set`, the launch file or the
  `/set_speed_limits` service.

## Bonus: bag file

A run of the 3 m square (with `max_linear_speed:=1.5`) is stored in
[turtle_bringup/bag/square_run](turtle_bringup/bag/square_run).

Record:
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag record -o square_run --topics /turtle1/cmd_vel /turtle1/pose
# in another terminal:
ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}"
# Ctrl+C the recorder when the square is done
```

`ros2 bag info turtle_bringup/bag/square_run`:
```
Storage id:        mcap
Duration:          23.904832507s
Messages:          2150
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 655
                   Topic: /turtle1/pose | Type: turtlesim_msgs/msg/Pose | Count: 1495
```

Play back (with only turtlesim running, or the full system idle):
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger     # the replay must start from the same pose
ros2 bag play turtle_bringup/bag/square_run --topics /turtle1/cmd_vel
```
Only `cmd_vel` is replayed, because turtlesim publishes its own pose. In our replay the turtle
drew the same square: x in [5.544, 8.542], y in [5.544, 8.555], against the expected [5.544, 8.544].
It finished about 8 cm from the start, which is normal drift for an open-loop replay.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `symbol lookup error: ... undefined symbol: has_buffer_fields_unique_identifier_msgs__msg__UUID` | The ROS install is only partly upgraded (new code generator, old message libraries). Run `sudo apt update && sudo apt full-upgrade`, then do a clean rebuild (`rm -rf build install log && colcon build`). |
| `Setting parameter failed: Wrong parameter type` | Speeds are doubles: write `1.0`, not `1`. |
| Goal is rejected | A goToPose goal is already running. Wait for it to finish or cancel it. |
