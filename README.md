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
   - Node automatically shuts down after task completion

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
2. After rotation completes, calls `/approach_shelf` service asynchronously
3. Passes `final_approach` parameter value to service request
4. Uses callback-based approach to handle service response without blocking
5. Automatically shuts down node after service completes

**Implementation Notes**:
- Uses async service client with response callback to avoid blocking executor
- Callback sets state to COMPLETED when service response is received
- Node terminates cleanly after task completion

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

---

# Checkpoint 10: Component Composition

## Overview

Checkpoint 10 builds on Checkpoint 9 by converting the attach_shelf functionality into ROS2 components using `rclcpp_components`. This demonstrates efficient node composition through shared library components, intra-process communication, and dynamic component loading.

## Package Information

- **Package Name**: `my_components`
- **Build Type**: `ament_cmake`
- **ROS2 Version**: Humble
- **Dependencies**: `rclcpp`, `rclcpp_components`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `attach_shelf`

## Task 1: PreApproach Component

### Description

The PreApproach component replicates the behavior of the `pre_approach.cpp` node from Checkpoint 9 as a composable component. It moves the robot forward until an obstacle is detected, then rotates to face the shelf.

### Component Details

**Component Class**: `my_components::PreApproach`

**Component Plugin**: `my_components::PreApproach`

**Node Name**: `pre_approach`

**Subscribed Topics**:
- `/scan` (`sensor_msgs/msg/LaserScan`) - Laser scan for obstacle detection
- `/diffbot_base_controller/odom` (`nav_msgs/msg/Odometry`) - Odometry for rotation tracking

**Published Topics**:
- `/diffbot_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`) - Velocity commands

**Parameters**:
- `shutdown_on_complete` (bool, default: true) - Whether to shutdown ROS2 when maneuver completes

**Hardcoded Parameters** (different from Checkpoint 9):
- Obstacle distance: **0.3 meters** (hardcoded, not a parameter)
- Rotation angle: **-90 degrees** (hardcoded, not a parameter)

**Control Parameters**:
- Linear velocity: 0.5 m/s
- Angular velocity: 0.8 rad/s
- Angle tolerance: 0.02 rad (~1.1 degrees)

### Implementation Details

The component implements the same state machine as Checkpoint 9:

1. **MOVING_FORWARD**
   - Robot moves forward at 0.5 m/s
   - Monitors laser scan center readings (±10 readings averaged)
   - Transitions to ROTATING when obstacle detected at 0.3m

2. **ROTATING**
   - Robot rotates -90 degrees (right turn)
   - Uses odometry feedback for precise angle tracking
   - Proportional control for smooth deceleration
   - Transitions to COMPLETED when angle error < 0.02 rad

3. **COMPLETED**
   - Robot stops all movement
   - If `shutdown_on_complete=true`: calls `rclcpp::shutdown()`
   - If `shutdown_on_complete=false`: remains ready for next task

**Key Difference from Regular Node**:
- Constructor accepts `rclcpp::NodeOptions` for component composition
- No `main()` function - component is loaded into a container
- Registered using `RCLCPP_COMPONENTS_REGISTER_NODE` macro
- Built as shared library for dynamic loading

### Building the Package

```bash
cd /ros2_ws
colcon build --packages-select my_components attach_shelf
source install/setup.bash
```

### Running Task 1

The launch file creates a component container and loads the PreApproach component:

```bash
ros2 launch my_components pre_approach_component.launch.py
```

This will:
- Create a container named `pre_approach_container`
- Load PreApproach component with `shutdown_on_complete=true`
- Robot moves forward until obstacle at 0.3m
- Robot rotates -90 degrees (right turn)
- Program automatically shuts down when complete

### RViz Visualization

**Note**: RViz cannot be launched together with components in the same launch file.

Launch RViz manually in a separate terminal:

```bash
# Source workspace first
source install/setup.bash

# Option 1: Using package prefix (recommended)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_components)/share/my_components/rviz/config.rviz

# Option 2: Using full path
cd /ros2_ws
ros2 run rviz2 rviz2 -d install/my_components/share/my_components/rviz/config.rviz
```

The RViz config file provides the same visualization as Checkpoint 9:
- Grid reference
- TF frames
- LaserScan with intensity coloring

### Expected Behavior

1. Component container starts
2. PreApproach component loads
3. Robot moves forward at 0.5 m/s
4. When obstacle detected at 0.3m, robot stops
5. Robot rotates -90 degrees (right turn) to face shelf
6. Proportional control ensures smooth stopping near target
7. Robot reaches final position (±1.1° precision)
8. Program automatically shuts down (`rclcpp::shutdown()`)

### Component Discovery

```bash
# List available components
ros2 component types | grep my_components

# List running containers
ros2 component list
```

### Grading Criteria (Task 1)

- **RViz config file present**: Located at `rviz/config.rviz` ✓
- **RViz config loads properly**: Contains Grid, TF, LaserScan displays with proper view angle ✓
- **Program termination**: Automatically shuts down when complete ✓

## Task 2: Final Approach Components

### Description

Task 2 converts the approach service server and creates a new service client as composable components. This demonstrates both manual composition (AttachServer) and runtime composition (AttachClient).

### Components Overview

1. **PreApproach** - Pre-approach maneuver (reused from Task 1)
2. **AttachServer** - Service server for shelf detection and attachment
3. **AttachClient** - Service client that calls the approach service

### Component 1: AttachServer

**Component Class**: `my_components::AttachServer`

**Component Plugin**: `my_components::AttachServer`

**Node Name**: `attach_server`

**Component Type**: Manual composition capable (can be used in manual composition executables)

**Service Provided**:
- `/approach_shelf` (`attach_shelf/srv/GoToLoading`) - Shelf detection and attachment service

**Subscribed Topics**:
- `/scan` (`sensor_msgs/msg/LaserScan`) - Laser intensity-based shelf detection
- `/diffbot_base_controller/odom` (`nav_msgs/msg/Odometry`) - Position/orientation tracking

**Published Topics**:
- `/diffbot_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`) - Velocity commands
- `/elevator_up` (`std_msgs/msg/String`) - Shelf lifting command

**Published TF**:
- `cart_frame` (static transform, child of `odom`) - Shelf position in odom frame

**Implementation Details**:

Identical functionality to `approach_service_server.cpp` from Checkpoint 9:

- **Shelf Leg Detection**: Uses laser scan intensities (≥8000 threshold) to detect reflective plates
- **TF2 Integration**: Publishes `cart_frame` static transform using temporal synchronization
- **Multi-threaded Executor**: Service callback and timer callback run in parallel threads
- **State Machine**: MOVING_TO_CART → MOVING_UNDER_SHELF → LIFTING_SHELF → COMPLETED

**Key Addition**:
- When state reaches COMPLETED (after shelf is lifted), calls `rclcpp::shutdown()` to terminate the container
- This ensures clean program termination after full workflow completes

### Component 2: AttachClient

**Component Class**: `my_components::AttachClient`

**Component Plugin**: `my_components::AttachClient`

**Node Name**: `attach_client`

**Component Type**: Runtime composition (loaded dynamically via `ros2 component load`)

**Service Called**:
- `/approach_shelf` (`attach_shelf/srv/GoToLoading`) - Calls with `attach_to_shelf=true`

**Implementation Details**:

- **Automatic Service Call**: Calls `/approach_shelf` service 5 seconds after component starts
- **Single Call**: Only calls service once (not repeated)
- **Service Request**: `attach_to_shelf = true`
- **Async Client**: Non-blocking service call with response callback
- **No Shutdown**: Client does not call `rclcpp::shutdown()` (server handles termination)

**Why 5 Second Delay?**
- Allows PreApproach maneuver to complete
- Ensures AttachServer is ready to process request
- Provides time for robot to reach loading position

### Building the Package

```bash
cd /ros2_ws
colcon build --packages-select my_components attach_shelf
source install/setup.bash
```

### Running Task 2

**Complete Workflow** (Two Terminal Approach):

**Terminal 1: Launch Container with PreApproach and AttachServer**

```bash
source install/setup.bash
ros2 launch my_components attach_to_shelf.launch.py
```

This creates container `my_container` with:
- **PreApproach component** (`shutdown_on_complete=false`) - Does NOT terminate when complete
- **AttachServer component** - Provides `/approach_shelf` service

**Terminal 2: Load AttachClient Dynamically (Runtime Composition)**

Wait for the robot to complete pre-approach maneuver (forward + rotation), then:

```bash
source install/setup.bash
ros2 component load /my_container my_components my_components::AttachClient
```

This dynamically loads the AttachClient component into the running container:
- AttachClient calls `/approach_shelf` service after 5 seconds
- AttachServer detects shelf legs and performs final approach
- Robot aligns with cart, moves underneath, lifts shelf
- AttachServer calls `rclcpp::shutdown()` when COMPLETED
- Entire container terminates cleanly

### Launch File Details

[attach_to_shelf.launch.py](my_components/launch/attach_to_shelf.launch.py):

```python
container = ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',  # Multi-threaded for callback groups
    composable_node_descriptions=[
        ComposableNode(
            package='my_components',
            plugin='my_components::PreApproach',
            name='pre_approach',
            parameters=[{'shutdown_on_complete': False}]),  # CRITICAL: False for Task 2
        ComposableNode(
            package='my_components',
            plugin='my_components::AttachServer',
            name='attach_server'),
    ],
    output='screen',
)
```

**Key Points**:
- Container executable: `component_container_mt` (multi-threaded executor required for AttachServer's callback groups)
- Container name: `my_container` (required for runtime composition)
- PreApproach has `shutdown_on_complete=False` (different from Task 1)
- AttachServer loaded at launch time
- AttachClient NOT in launch file (loaded dynamically at runtime)

**Why Multi-Threaded Container?**
- AttachServer uses separate callback groups for service and timer callbacks
- Service callback blocks while waiting for approach to complete
- Multi-threaded executor allows timer callback to run concurrently
- Single-threaded executor would cause deadlock

### RViz Visualization

Launch RViz manually (same as Task 1):

```bash
source install/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_components)/share/my_components/rviz/config.rviz
```

You should see:
- Robot movement during pre-approach
- Laser scan intensities (bright spots on shelf legs)
- `cart_frame` TF appear after shelf detection
- Robot navigation to cart position
- Final approach and attachment

### Expected Behavior (Complete Workflow)

**Phase 1: Pre-Approach** (Automatic)
1. Container starts with PreApproach and AttachServer components
2. PreApproach moves robot forward until obstacle at 0.3m
3. Robot rotates -90 degrees (right turn)
4. PreApproach completes but does NOT shutdown (stays ready)
5. Robot is now facing the shelf in loading position

**Phase 2: Runtime Component Loading** (Manual)
6. User loads AttachClient component dynamically
7. AttachClient waits 5 seconds, then calls `/approach_shelf` service

**Phase 3: Final Approach** (Automatic)
8. AttachServer detects shelf legs using laser intensities
9. Publishes `cart_frame` TF at midpoint between legs
10. Robot aligns with cart (rotation phase)
11. Robot moves to cart position (forward motion)
12. Robot advances 30cm underneath shelf
13. Robot lifts shelf (publishes to `/elevator_up`)
14. AttachServer reaches COMPLETED state
15. AttachServer calls `rclcpp::shutdown()`
16. **Entire container terminates** (all three components shut down together)

### Shutdown Coordination

**Critical Design Pattern**:

- **PreApproach**: Does NOT call shutdown (uses `shutdown_on_complete=false` parameter)
- **AttachClient**: Does NOT call shutdown (logs service response only)
- **AttachServer**: DOES call shutdown in COMPLETED state (after shelf is lifted)

**Why This Pattern?**

All three components run in the same container (`my_container`). If any component calls `rclcpp::shutdown()`, the entire process terminates. Therefore:

- PreApproach must NOT shutdown (would terminate before service can be called)
- AttachClient must NOT shutdown (would terminate before server completes)
- AttachServer must shutdown when COMPLETED (ensures clean termination after full workflow)

This ensures the program terminates only after the complete workflow succeeds.

### Component Discovery and Inspection

```bash
# List available components
ros2 component types | grep my_components

# Expected output:
#   my_components
#     my_components::PreApproach
#     my_components::AttachServer
#     my_components::AttachClient

# List running containers
ros2 component list

# Expected output:
#   /my_container
#      1  /pre_approach
#      2  /attach_server
#      3  /attach_client  (after manual loading)

# Load AttachClient into running container
ros2 component load /my_container my_components my_components::AttachClient

# Expected output:
#   Loaded component 3 into '/my_container' container node as '/attach_client'

# Unload a component (if needed for testing)
ros2 component unload /my_container 3
```

### Grading Criteria (Task 2)

1. **Robot moves towards wall and stops facing shelf** (2.0 points) ✓
   - PreApproach component executes forward movement + rotation
   - Same behavior as Checkpoint 9

2. **/approach_shelf service is ready** (1.0 point) ✓
   - AttachServer component provides service
   - Service available immediately after launch

3. **AttachClient loads and robot does final approach** (1.5 points) ✓
   - Runtime composition with `ros2 component load`
   - Service call triggers final approach state machine
   - Robot aligns with and moves to cart position

4. **Robot lifts shelf after final approach** (1.0 point) ✓
   - LIFTING_SHELF state publishes to `/elevator_up`
   - Same behavior as Checkpoint 9

5. **Valid .rviz config file is present in package** (0.5 points) ✓
   - Located at `rviz/config.rviz`
   - Installed via CMakeLists.txt

6. **RViz loads with proper display elements and view angle** (0.5 points) ✓
   - Grid, TF frames, LaserScan displays
   - Proper camera positioning

7. **The program terminates when the node processes are complete** (0.5 points) ✓
   - AttachServer calls `rclcpp::shutdown()` when COMPLETED
   - Entire container terminates after shelf attachment

**Total**: 7.0 points

### Benefits of Component Composition

**Compared to Checkpoint 9 (separate nodes)**:

1. **Reduced Memory Footprint**: All components share the same process space
2. **Faster Communication**: Intra-process communication uses zero-copy message passing
3. **Lower Latency**: No serialization/deserialization overhead between components
4. **Easier Deployment**: Single container manages multiple components
5. **Better Resource Utilization**: Shared executor and middleware

**Runtime Composition Benefits**:

- Dynamic loading/unloading of components without restarting container
- Flexible system configuration
- Testing individual components independently
- Modular architecture for complex systems

### Component vs Node Comparison

| Feature | Regular Node (Checkpoint 9) | Component (Checkpoint 10) |
|---------|----------------------------|--------------------------|
| Entry Point | `main()` function | `rclcpp::NodeOptions` constructor |
| Loading | Launched as separate process | Loaded into container |
| Communication | Inter-process (DDS) | Intra-process (zero-copy) |
| Memory | Separate process memory | Shared process memory |
| Registration | Executable in CMakeLists | `RCLCPP_COMPONENTS_REGISTER_NODE` |
| Library Type | Executable | Shared library (`.so`) |
| Dynamic Loading | Not possible | `ros2 component load` |

### File Structure

```
my_components/
├── CMakeLists.txt                          # Build configuration with component registration
├── package.xml                             # Package metadata and dependencies
├── include/my_components/
│   ├── pre_approach_component.hpp          # PreApproach component header
│   ├── attach_server_component.hpp         # AttachServer component header
│   ├── attach_client_component.hpp         # AttachClient component header
│   └── visibility_control.h                # Component visibility macros
├── src/
│   ├── pre_approach_component.cpp          # PreApproach implementation
│   ├── attach_server_component.cpp         # AttachServer implementation (manual composition)
│   └── attach_client_component.cpp         # AttachClient implementation (runtime composition)
├── launch/
│   ├── pre_approach_component.launch.py    # Task 1: PreApproach standalone
│   └── attach_to_shelf.launch.py           # Task 2: Container with PreApproach + AttachServer
├── rviz/
│   └── config.rviz                         # RViz configuration (same as Checkpoint 9)
└── README.md                                # Package documentation
```

### Testing Tips

1. **Test Task 1 First**:
   ```bash
   ros2 launch my_components pre_approach_component.launch.py
   ```
   Verify program terminates after rotation completes.

2. **Test Task 2 Container Launch**:
   ```bash
   ros2 launch my_components attach_to_shelf.launch.py
   ```
   Verify PreApproach completes but does NOT terminate.
   Verify `/approach_shelf` service is available:
   ```bash
   ros2 service list | grep approach_shelf
   ```

3. **Test Runtime Composition**:
   ```bash
   ros2 component load /my_container my_components my_components::AttachClient
   ```
   Verify component loads, service is called, and final approach executes.

4. **Monitor Component Lifecycle**:
   ```bash
   # Terminal 1: Launch container
   ros2 launch my_components attach_to_shelf.launch.py

   # Terminal 2: Monitor components
   ros2 component list

   # Terminal 3: Load AttachClient
   ros2 component load /my_container my_components my_components::AttachClient

   # Terminal 4: Monitor service calls
   ros2 service echo /approach_shelf
   ```

5. **RViz Visualization**:
   Launch RViz before running Task 2 to see the complete workflow:
   ```bash
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_components)/share/my_components/rviz/config.rviz
   ```

### Common Issues and Solutions

**Issue**: Container terminates immediately after PreApproach completes
- **Solution**: Check `shutdown_on_complete` parameter is `false` in attach_to_shelf.launch.py

**Issue**: "Service /approach_shelf not available"
- **Solution**: Wait for AttachServer to fully initialize (3-5 seconds after launch)

**Issue**: AttachClient loads but nothing happens
- **Solution**: Wait 5 seconds - AttachClient has built-in delay before calling service

**Issue**: Program doesn't terminate after shelf attachment
- **Solution**: Verify AttachServer calls `rclcpp::shutdown()` in COMPLETED state

**Issue**: RViz shows "Fixed Frame [odom] does not exist"
- **Solution**: Ensure robot simulation is running and TF tree is publishing

### Additional Resources

For more information on ROS2 component composition:
- ROS2 Components Tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html
- rclcpp_components package: https://github.com/ros2/rclcpp/tree/humble/rclcpp_components

For Checkpoint 9 implementation details (pre-components):
- See Task 1 and Task 2 sections above in this README
