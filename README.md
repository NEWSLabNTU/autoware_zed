# Autoware ZED

This ROS 2 package transforms ZED camera object detection messages (`zed_msgs/msg/ObjectsStamped`) to Autoware perception messages (`autoware_perception_msgs/msg/DetectedObjects`).

## Features

- Converts ZED object detection results to Autoware format
- Supports multi-camera setups
- Maps ZED object labels to Autoware classifications
- Preserves position, velocity, and bounding box information
- Configurable confidence thresholds
- Validates numeric values to prevent NaN/Inf in output

## Prerequisites

This package depends on the ZED ROS 2 packages. You must build and source them before building this package.

## Installation

### Step 1: Build ZED Dependencies

First, build the ZED ROS 2 packages:

```bash
cd ~/ros2_ws/src/zed
make build
# or from workspace root:
# cd ~/ros2_ws
# make build-zed
```

### Step 2: Build Autoware ZED Package

After the ZED packages are built, source their installation and build this package:

```bash
cd ~/ros2_ws/src/autoware_zed
# Source the ZED packages
source ../zed/install/setup.sh
# Build this package
make build
# or from workspace root:
# cd ~/ros2_ws
# make build-autoware
```

### Step 3: Source the Installation

```bash
cd ~/ros2_ws
source install/setup.bash
```

## Message Mapping

### Object Classification Mapping
- `PERSON` → `PEDESTRIAN`
- `VEHICLE/CAR` → `CAR`
- `TRUCK` → `TRUCK`
- `BUS` → `BUS`
- `BICYCLE` → `BICYCLE`
- `MOTORCYCLE` → `MOTORCYCLE`
- Other labels → `UNKNOWN`

### Data Transformation
- **Position**: Direct mapping from ZED position
- **Velocity**: Used if tracking is available and enabled
- **Bounding Box**: ZED 3D dimensions mapped to Autoware shape
- **Confidence**: ZED confidence (0-100) converted to probability (0.0-1.0)
- **Frame ID**: Preserved from input message

## Usage

### Single Camera
```bash
ros2 launch autoware_zed single_camera.launch.xml \
  input_topic:=/zed/zed_node/obj_det/objects \
  output_topic:=/perception/object_recognition/objects
```

### Multi-Camera Setup (Default)
```bash
ros2 launch autoware_zed autoware_zed.launch.xml
```

This will create two transformer nodes:
- Front camera: `/zed_multi/zed_front/obj_det/objects` → `/perception/object_recognition/objects/front`
- Back camera: `/zed_multi/zed_back/obj_det/objects` → `/perception/object_recognition/objects/back`

### Running the Node Directly
```bash
ros2 run autoware_zed autoware_zed_node
```

## Parameters

- `input_topic` (string): Input topic with ZED objects
- `output_topic` (string): Output topic for Autoware objects
- `target_frame` (string, default: "base_link"): Target frame for transformed objects
- `use_tracking_velocity` (bool, default: true): Use velocity from ZED tracking
- `existence_probability_threshold` (double, default: 0.5): Minimum confidence threshold

## Visualization

To visualize the transformed objects in RViz2:
1. Add a `MarkerArray` display
2. Set topic to your output topic (e.g., `/perception/object_recognition/objects/front`)
3. The objects will be displayed as 3D bounding boxes

## Integration with Your System

To use with your multi-camera ZED setup:

1. Ensure ZED packages are built:
   ```bash
   cd ~/ros2_ws
   make build-zed
   ```

2. Make sure cameras are running with object detection:
   ```bash
   ./run.sh
   ```

3. Launch the transformer:
   ```bash
   # From workspace root after sourcing
   ros2 launch autoware_zed autoware_zed.launch.xml
   # Or using the Makefile
   make run-transformer
   ```

4. The Autoware-formatted objects will be published on:
   - `/perception/object_recognition/objects/front`
   - `/perception/object_recognition/objects/back`

## Testing

To test the converter:
```bash
# Terminal 1: Run ZED cameras
cd ~/ros2_ws
./run.sh

# Terminal 2: Run the transformer
cd ~/ros2_ws
source install/setup.bash
ros2 launch autoware_zed autoware_zed.launch.xml

# Terminal 3: Monitor the output
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /perception/object_recognition/objects/front
```

Or use the provided test script:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/autoware_zed/scripts/test_converter.py
```