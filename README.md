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

## Task 2: Final Approach and Shelf Attachment

### Description

Task 2 extends the pre-approach functionality with a service-based architecture that detects the shelf legs using laser scan intensities, publishes a TF frame at the cart position, and optionally performs the final approach to attach to the shelf.

### Custom Service

**Service File**: `srv/GoToLoading.srv`

```
# Request
bool attach_to_shelf
---
# Response
bool complete
```

- `attach_to_shelf`: `false` = only publish cart_frame TF, `true` = perform full attachment sequence
- `complete`: Service result (true if successful, false if failed)

### Nodes

#### 1. Approach Service Server Node

**Node Name**: `approach_service_server_node`

**Service Provided**:
- `/approach_shelf` (`attach_shelf/srv/GoToLoading`) - Main service for shelf detection and attachment

**Subscribed Topics**:
- `/scan` (`sensor_msgs/msg/LaserScan`) - Laser scan data for shelf leg detection
- `/diffbot_base_controller/odom` (`nav_msgs/msg/Odometry`) - Robot position and orientation

**Published Topics**:
- `/diffbot_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`) - Velocity commands
- `/elevator_up` (`std_msgs/msg/String`) - Command to lift the shelf

**Published TF**:
- `cart_frame` (static transform) - Position of cart in `odom` frame

**Implementation Details**:

**Shelf Leg Detection** (`detect_shelf_legs()`):
- Scans laser intensities for reflective plates (threshold: ≥ 8000)
- Groups consecutive high-intensity readings (minimum 5 consecutive readings)
- Each group represents one shelf leg
- Returns vector of detected legs with their angle and range

**TF Broadcasting** (`publish_cart_frame()`):
- Calculates midpoint between two detected legs in laser frame
- Uses TF2 `transform()` to convert point from `robot_front_laser_base_link` to `odom` frame
- Publishes static transform `cart_frame` as child of `odom`
- This allows RViz to visualize the cart position in the global frame
- Uses laser scan timestamp to ensure proper temporal synchronization with TF tree

**Final Approach State Machine** (if `attach_to_shelf=true`):

1. **MOVING_TO_CART**
   - Uses TF2 `lookupTransform()` to get real-time transform from `robot_base_link` to `cart_frame`
   - Two-phase control strategy:
     - **Far from target** (>30cm and >6° off): Pure rotation to align
     - **Close to target**: Forward motion with simultaneous angular correction
   - Transitions when distance < 10cm to account for imperfect alignment
   - Proportional control for smooth, adaptive approach

2. **MOVING_UNDER_SHELF**
   - Advances exactly 30cm forward from cart position
   - Tracks distance using odometry
   - Moves at constant 0.2 m/s

3. **LIFTING_SHELF**
   - Publishes message to `/elevator_up` topic
   - Triggers shelf lifting mechanism

4. **COMPLETED**
   - Returns service response (complete=true)
   - Robot is now attached to shelf

**Control Parameters**:

- Distance threshold for state transition: 0.10 m (10 cm)
- Far alignment threshold: 0.3 m distance, 0.1 rad (~6°) angle
- Under-shelf distance: 0.30 m (30 cm)
- Control loop frequency: 10 Hz

**Concurrency Architecture**:

- Uses `MultiThreadedExecutor` with separate callback groups
- Service callback and timer callback run in parallel threads
- Prevents deadlock where service blocks executor from running control loop

#### 2. Pre-Approach V2 Node

**Node Name**: `pre_approach_v2_node`

An extended version of the pre_approach_node that calls the approach service after completing the rotation.

**Parameters**:
- `obstacle` (double, default: 0.3) - Distance to obstacle in meters
- `degrees` (int, default: -90) - Rotation angle in degrees
- `final_approach` (bool, default: false) - Whether to call service with attach_to_shelf=true

**Behavior**:
1. Executes same pre-approach logic as Task 1
2. After rotation completes, calls `/approach_shelf` service
3. Passes `final_approach` parameter value to service request
4. Waits for service response and logs result

### Building the Package

```bash
cd /ros2_ws
colcon build --packages-select attach_shelf
source install/setup.bash
```

### Running Task 2

The `attach_to_shelf.launch.py` file launches the complete system with RViz visualization.

**TF Publishing Only** (detect shelf and publish cart_frame, no attachment):

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py final_approach:=false
```

This will:
- Move robot forward to obstacle
- Rotate to face shelf
- Detect shelf legs using laser intensities
- Publish `cart_frame` static TF in odom frame
- Return without moving robot

**Full Shelf Attachment** (complete sequence):

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py final_approach:=true
```

This will:
- Execute pre-approach (move forward + rotate)
- Detect shelf legs
- Publish cart_frame TF
- Align with cart position
- Move to cart
- Advance 30cm under shelf
- Lift shelf via `/elevator_up`

**Custom Parameters**:

```bash
# Stop at 40cm, turn right 90°, attach to shelf
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=true

# Stop at 50cm, turn left 90°, TF only
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.5 degrees:=90 final_approach:=false
```

### Launch File Details

The [attach_to_shelf.launch.py](phase_2/section_9/checkpoint/checkpoint9/attach_shelf/launch/attach_to_shelf.launch.py) file launches:

1. **approach_service_server_node** - Service server for shelf detection and attachment
2. **rviz2** - Visualization with the same config as Task 1
3. **pre_approach_v2_node** - Pre-approach with service client integration (delayed 3 seconds)

**Launch Timing**:

- The `pre_approach_v2_node` is delayed by 3 seconds using `TimerAction`
- This allows the TF tree to fully stabilize before robot movement begins
- Ensures `approach_service_server_node` is ready and TF transforms are available
- Prevents TF lookup errors during initial startup

```python
ros2 launch attach_shelf attach_to_shelf.launch.py \
    obstacle:=0.3 \
    degrees:=-90 \
    final_approach:=true
```

### Expected Behavior

**With final_approach:=false** (TF publishing only):
1. Robot moves forward until obstacle detected
2. Robot rotates to face shelf
3. Laser scan detects reflective plates on shelf legs (intensity ≥ 8000)
4. System calculates cart position and publishes static TF
5. RViz shows `cart_frame` in TF display
6. Robot remains stationary
7. Service returns complete=true

**With final_approach:=true** (Full attachment):
1. Executes steps 1-3 above
2. Publishes cart_frame TF
3. Robot aligns with cart (rotates to face it directly)
4. Robot moves forward to cart position
5. Robot advances 30cm underneath shelf
6. Robot publishes to `/elevator_up` to lift shelf
7. Service returns complete=true
8. Robot is attached to shelf

### RViz Visualization

The launch file loads the same RViz configuration as Task 1, showing:
- Grid reference
- TF frames (including the new `cart_frame`)
- LaserScan with intensity coloring (shelf legs appear bright)

You should see the `cart_frame` appear in RViz after the shelf legs are detected. The frame will be positioned at the midpoint between the two detected legs in the odom frame.

### Laser Scan Intensity Detection

The shelf legs have reflective plates that produce high-intensity laser readings:

- **Normal readings**: intensity < 8000
- **Reflective plates**: intensity ≥ 8000
- **Minimum consecutive readings**: 5 (to filter noise)

The algorithm groups consecutive high-intensity readings into "legs" and requires at least 2 legs for a valid shelf detection.

### Testing Tips

- Monitor laser scan intensities in RViz (rainbow coloring shows high intensities as red/bright)
- Check `/approach_shelf` service logs for detection results
- Verify `cart_frame` appears in RViz TF display after detection
- Test with `final_approach:=false` first to verify TF publishing
- Then test with `final_approach:=true` for full sequence

### Key Implementation Details

**TF2 Integration**:

- Uses TF2 `transform()` for point transformation from laser frame to odom frame
- Uses TF2 `lookupTransform()` for real-time navigation during final approach
- Properly synchronized using laser scan timestamps instead of wall clock time
- Avoids manual coordinate transformation mathematics which can accumulate errors

**Temporal Synchronization**:

- All TF operations use `last_laser_scan_->header.stamp` for timestamp
- Ensures TF lookups reference the correct point in time
- Prevents "extrapolation into the future" errors
- Static TF broadcast includes 500ms propagation delay + verification check

**Robust Navigation**:

- Adaptive two-phase control prevents oscillation near target
- Relaxed tolerances when close to goal (10cm vs 5cm)
- Simultaneous linear and angular control for efficient approach
- Proportional control for smooth deceleration

## File Structure

```
attach_shelf/
├── CMakeLists.txt                       # Build configuration with service generation
├── package.xml                          # Package metadata and dependencies
├── srv/
│   └── GoToLoading.srv                 # Custom service definition
├── launch/
│   ├── pre_approach.launch.xml         # Task 1 launch file
│   └── attach_to_shelf.launch.py       # Task 2 launch file with service integration
├── rviz/
│   └── config.rviz                     # RViz configuration file
├── src/
│   ├── pre_approach.cpp                # Task 1: Basic pre-approach node
│   ├── pre_approach_v2.cpp             # Task 2: Pre-approach with service client
│   └── approach_service_server.cpp     # Task 2: Service server for shelf attachment
└── include/
    └── attach_shelf/                   # Header files (future use)
```
