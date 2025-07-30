# Launch Files

This directory contains both Python (.launch.py) and XML (.launch.xml) launch files. Both formats provide the same functionality.

## XML Launch Files (Recommended)

### Multi-Camera Setup
```bash
ros2 launch zed_to_autoware_transformer zed_to_autoware_transformer.launch.xml
```

With custom namespaces:
```bash
ros2 launch zed_to_autoware_transformer zed_to_autoware_transformer.launch.xml \
  front_camera_ns:=/custom/front \
  back_camera_ns:=/custom/back
```

### Single Camera
```bash
ros2 launch zed_to_autoware_transformer single_camera.launch.xml \
  input_topic:=/zed/zed_node/obj_det/objects \
  output_topic:=/perception/object_recognition/objects
```

### Test Front Camera
```bash
ros2 launch zed_to_autoware_transformer test_front_camera.launch.xml
```

## Python Launch Files (Alternative)

The Python launch files have the same names with .launch.py extension and accept the same arguments:

```bash
ros2 launch zed_to_autoware_transformer zed_to_autoware_transformer.launch.py
ros2 launch zed_to_autoware_transformer single_camera.launch.py
ros2 launch zed_to_autoware_transformer test_front_camera.launch.py
```

## Common Arguments

- `use_sim_time`: Use simulation clock (default: false)
- `input_topic`: Input topic with ZED ObjectsStamped messages
- `output_topic`: Output topic for Autoware DetectedObjects messages
- `front_camera_ns`: Namespace for front camera (multi-camera only)
- `back_camera_ns`: Namespace for back camera (multi-camera only)