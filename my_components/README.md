# my_components Package

This package contains ROS2 component implementations for Checkpoint 10.

## Task 1: PreApproach Component

The PreApproach component replicates the behavior of the `pre_approach.cpp` node from Checkpoint 9, moving the robot in front of the shelf and facing it.

### Component Features

- **Hardcoded parameters**:
  - Obstacle distance: 0.3 meters
  - Rotation angle: -90 degrees (turn right)
- **State machine**: MOVING_FORWARD → ROTATING → COMPLETED
- **Automatic shutdown**: Program terminates when maneuver is complete

### Build Instructions

```bash
# Build the package
colcon build --packages-select my_components

# Source workspace
source install/setup.bash
```

### Launch Instructions

#### Launch Component

```bash
ros2 launch my_components pre_approach_component.launch.py
```

#### Launch RViz (Separate Terminal)

**Note**: RViz cannot be launched together with components. Launch it manually in a separate terminal:

```bash
# Option 1: Using package prefix
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_components)/share/my_components/rviz/config.rviz

# Option 2: Using full path
cd /path/to/ros2_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d install/my_components/share/my_components/rviz/config.rviz
```

### Expected Behavior

1. The robot moves forward at 0.5 m/s
2. When an obstacle is detected at 0.3m, the robot stops
3. The robot rotates -90 degrees (right turn) to face the shelf
4. The program automatically shuts down when complete

### Files Structure

```
my_components/
├── include/my_components/
│   ├── pre_approach_component.hpp    # Component header
│   └── visibility_control.h          # Component visibility macros
├── src/
│   └── pre_approach_component.cpp    # Component implementation
├── launch/
│   └── pre_approach_component.launch.py  # Launch file
├── rviz/
│   └── config.rviz                   # RViz configuration
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Component Information

- **Package**: `my_components`
- **Component Plugin**: `my_components::PreApproach`
- **Node Name**: `pre_approach`
- **Namespace**: `my_components`

### Topics Used

- **Subscriptions**:
  - `/scan` (sensor_msgs/LaserScan) - Laser scan for obstacle detection
  - `/diffbot_base_controller/odom` (nav_msgs/Odometry) - Odometry for rotation tracking

- **Publications**:
  - `/diffbot_base_controller/cmd_vel_unstamped` (geometry_msgs/Twist) - Velocity commands

### Grading Criteria

- ✅ **RViz config file present**: Located at `rviz/config.rviz`
- ✅ **RViz config loads properly**: Contains all required display elements and view angle
- ✅ **Program termination**: Automatically shuts down when node processes are complete

---

## Task 2: Final Approach Components

### AttachServer Component

The AttachServer component contains the `/approach_shelf` service server from Checkpoint 9. When called, it performs the final approach behavior to attach the robot to the shelf.

**Component Type**: Manual Composition (can be used in manual composition executables)

**Features**:
- Service: `/approach_shelf` (attach_shelf/srv/GoToLoading)
- Shelf leg detection using laser intensity (≥8000 threshold)
- TF2 integration: publishes `cart_frame` static transform
- Multi-threaded executor with callback groups
- State machine: MOVING_TO_CART → MOVING_UNDER_SHELF → LIFTING_SHELF → COMPLETED

### AttachClient Component

The AttachClient component contains a service client that calls the `/approach_shelf` service to initiate the final approach.

**Component Type**: Runtime Composition (loaded dynamically via launch file)

**Features**:
- Service client for `/approach_shelf`
- Automatically calls service 5 seconds after startup
- Sends request with `attach_to_shelf = true`
- Single service call (not repeated)

### Launch File: attach_to_shelf.launch.py

This launch file starts a container named `my_container` with two components:
1. **PreApproach** (name: `pre_approach`) - Moves robot in front of shelf
2. **AttachServer** (name: `attach_server`) - Provides approach service

```bash
ros2 launch my_components attach_to_shelf.launch.py
```

### Complete Workflow

**Terminal 1 - Launch main components:**
```bash
colcon build --packages-select my_components attach_shelf
source install/setup.bash
ros2 launch my_components attach_to_shelf.launch.py
```

**Terminal 2 - Load AttachClient dynamically (runtime composition):**
```bash
source install/setup.bash
ros2 component load /my_container my_components my_components::AttachClient
```

### Component Information

#### AttachServer
- **Package**: `my_components`
- **Component Plugin**: `my_components::AttachServer`
- **Node Name**: `attach_server`
- **Service Provided**: `/approach_shelf`

#### AttachClient
- **Package**: `my_components`
- **Component Plugin**: `my_components::AttachClient`
- **Node Name**: `attach_client`
- **Service Called**: `/approach_shelf`

### Topics & Services Used

**AttachServer Topics**:
- **Subscriptions**:
  - `/scan` (sensor_msgs/LaserScan) - Laser intensity-based shelf detection
  - `/diffbot_base_controller/odom` (nav_msgs/Odometry) - Position/orientation tracking
- **Publications**:
  - `/diffbot_base_controller/cmd_vel_unstamped` (geometry_msgs/Twist) - Velocity commands
  - `/elevator_up` (std_msgs/String) - Shelf lifting command
- **TF Published**:
  - `cart_frame` (static, child of `odom`) - Shelf position

**Services**:
- `/approach_shelf` (attach_shelf/srv/GoToLoading)
  - Request: `bool attach_to_shelf`
  - Response: `bool complete`

### Files Structure (Task 2)

```
my_components/
├── include/my_components/
│   ├── attach_server_component.hpp    # AttachServer header
│   ├── attach_client_component.hpp    # AttachClient header
│   └── ...
├── src/
│   ├── attach_server_component.cpp    # AttachServer implementation
│   ├── attach_client_component.cpp    # AttachClient implementation
│   └── ...
├── launch/
│   ├── attach_to_shelf.launch.py      # Main launch file
│   └── ...
└── ...
```
