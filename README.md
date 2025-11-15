# Checkpoint 9: Attach Shelf Package

## Overview

The `attach_shelf` package implements autonomous robot navigation for warehouse/logistics scenarios. The robot approaches a shelf structure, positions itself correctly, and prepares for attachment/docking operations.

## Package Information

- **Package Name**: `attach_shelf`
- **Build Type**: `ament_cmake`
- **ROS2 Version**: Humble
- **Dependencies**: `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`, `tf2_geometry_msgs`

## Task 1: Pre-Approach Node

### Description

The `pre_approach_node` enables the robot to navigate from its starting position to the loading position (facing the shelf). The node uses laser scan data to detect obstacles and executes a two-phase maneuver:

1. **Forward Movement**: Robot moves forward until it detects an obstacle at a specified distance
2. **Rotation**: Robot rotates by a specified number of degrees to face the shelf

### Node Details

**Node Name**: `pre_approach_node`

**Subscribed Topics**:
- `/scan` (`sensor_msgs/msg/LaserScan`) - Laser scan data for obstacle detection
- `/diffbot_base_controller/odom` (`nav_msgs/msg/Odometry`) - Odometry for precise rotation tracking

**Published Topics**:
- `/diffbot_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`) - Velocity commands

**Parameters**:
- `obstacle` (double, default: 0.3) - Distance in meters to obstacle at which robot stops
- `degrees` (int, default: -90) - Rotation angle in degrees after stopping (negative = right turn, positive = left turn)

**Control Parameters** (hardcoded):
- Linear velocity: 0.5 m/s
- Angular velocity: 0.8 rad/s
- Angle tolerance: 0.02 rad (~1.1 degrees)

### Implementation Details

The node implements a state machine with three states:

1. **MOVING_FORWARD**
   - Robot publishes linear velocity (0.5 m/s) to move forward
   - Continuously monitors laser scan center readings
   - Averages ±10 readings around center for robust detection
   - Transitions to ROTATING when obstacle distance ≤ `obstacle` parameter

2. **ROTATING**
   - Robot stops linear movement
   - Uses odometry feedback to track current yaw angle
   - Calculates target yaw based on initial yaw + desired rotation
   - Implements proportional control for smooth stopping near target
   - Rotation speed: 0.8 rad/s (full speed), proportional control within ~17° of target
   - Transitions to COMPLETED when angle error < 0.02 rad (~1.1°)

3. **COMPLETED**
   - Robot stops all movement
   - Final position: facing the shelf at loading position with high precision

### Building the Package

```bash
cd /ros2_ws
colcon build --packages-select attach_shelf
source install/setup.bash
```

### Running the Node

The launch file automatically starts both the pre_approach node and RViz2 with a pre-configured visualization setup.

**Default parameters** (stop at 30cm, turn right 90°):

```bash
ros2 launch attach_shelf pre_approach.launch.xml
```

This will launch:
- `pre_approach_node` - Robot control node
- `rviz2` - Visualization with pre-configured camera angle and displays

**Custom parameters**:

```bash
# Stop at 50cm, turn right 90°
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.5 degrees:=-90

# Stop at 40cm, turn left 90°
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.4 degrees:=90

# Stop at 25cm, turn right 180°
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.25 degrees:=-180
```

### RViz Visualization

The launch file includes RViz2 with a custom configuration that provides:

**Display Elements**:

- **Grid** - Ground plane reference (1m cells)
- **TF Frames** - All robot transform frames with 0.5m scale markers
- **LaserScan** - Point cloud visualization with rainbow intensity coloring

**Camera Setup**:

- Fixed frame: `odom`
- View: Orbit camera at ~8m distance
- Angle: Elevated perspective (pitch: 0.36 rad, yaw: 0.16 rad)
- Provides clear view of robot movement, rotation, and laser scan data

The RViz config is automatically loaded from [rviz/config.rviz](phase_2/section_9/checkpoint/checkpoint9/attach_shelf/rviz/config.rviz).

### Expected Behavior

Starting from the initial position shown in the environment images:

1. Robot begins moving forward at 0.5 m/s
2. Laser scan monitors distance to wall/shelf ahead
3. When front obstacle detected at configured distance (e.g., 0.3m), robot stops
4. Robot uses odometry to precisely rotate by configured degrees (e.g., -90° for right turn)
5. Proportional control smoothly decelerates rotation near target angle
6. Robot ends facing the shelf structure in loading position (±1.1° precision)
7. Robot remains stationary, ready for next task phase

### Testing Tips

- Adjust `obstacle` parameter based on shelf distance in your environment
- Typical values: 0.25m to 0.5m
- Use negative `degrees` for right turn (most common for shelf approach)
- Use positive `degrees` for left turn
- Monitor logs for state transitions and distance readings

### Launch File Details

The [pre_approach.launch.xml](phase_2/section_9/checkpoint/checkpoint9/attach_shelf/launch/pre_approach.launch.xml) file launches both nodes:

```xml
<launch>
  <!-- Parameters -->
  <arg name="obstacle" default="0.3" description="Distance to obstacle in meters"/>
  <arg name="degrees" default="-90" description="Rotation degrees (negative=right)"/>

  <!-- Pre-approach node -->
  <node pkg="attach_shelf" exec="pre_approach_node" name="pre_approach_node" output="screen">
    <param name="obstacle" value="$(var obstacle)"/>
    <param name="degrees" value="$(var degrees)"/>
  </node>

  <!-- RViz visualization with config file -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen"
        args="-d $(find-pkg-share attach_shelf)/rviz/config.rviz"/>
</launch>
```

## File Structure

```
attach_shelf/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata and dependencies
├── launch/
│   └── pre_approach.launch.xml      # Launch file with parameters and RViz
├── rviz/
│   └── config.rviz                  # RViz configuration file
├── src/
│   └── pre_approach.cpp             # Pre-approach node implementation
└── include/
    └── attach_shelf/                # Header files (future use)
```
