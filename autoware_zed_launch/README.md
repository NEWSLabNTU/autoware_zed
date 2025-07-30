# Autoware ZED Launch

This package provides example launch files for integrating ZED cameras with Autoware using the autoware_zed_converter node.

## Launch Files

### 1. Single Camera Example (single_camera_example.launch.xml)
Complete example that launches a single ZED camera with object detection and converts the output to Autoware format.

**Arguments:**
- `camera_name`: Camera name (default: "zed")
- `camera_model`: Camera model (default: "zedxm")
- `camera_serial`: Camera serial number, 0 for auto-detect (default: 0)
- `converter_output_topic`: Output topic for Autoware objects (default: "/perception/object_recognition/objects")
- `use_sim_time`: Use simulation clock (default: false)

**Usage:**
```bash
# Launch with default settings
ros2 launch autoware_zed_launch single_camera_example.launch.xml

# Launch with custom camera
ros2 launch autoware_zed_launch single_camera_example.launch.xml \
  camera_name:=zed_front \
  camera_model:=zedxm \
  camera_serial:=12345678
```

### 2. Multi-Camera Example (multi_camera_example.launch.xml)
Complete example that launches multiple ZED cameras with object detection and converts outputs to Autoware format.

**Arguments:**
- `cam_names`: List of camera names (default: "[zed_front,zed_back]")
- `cam_models`: List of camera models (default: "[zedxm,zedxm]")
- `cam_serials`: List of camera serial numbers (default: "[59072113,50499972]")
- `use_sim_time`: Use simulation clock (default: false)

**Usage:**
```bash
# Launch with default settings (two ZED X Mini cameras)
ros2 launch autoware_zed_launch multi_camera_example.launch.xml

# Launch with custom cameras
ros2 launch autoware_zed_launch multi_camera_example.launch.xml \
  cam_names:='[left_cam,right_cam]' \
  cam_models:='[zed2,zed2]' \
  cam_serials:='[11111111,22222222]'
```

## Output Topics

- **Single Camera Example**: `/perception/object_recognition/objects`
- **Multi-Camera Example**: 
  - Front camera: `/perception/object_recognition/objects/front`
  - Back camera: `/perception/object_recognition/objects/back`

## Dependencies

- `autoware_zed_converter`: The converter node package
- `zed_launch`: Launch files for ZED cameras
- `zed_wrapper`: ZED ROS 2 wrapper
- `zed_multi_camera`: Multi-camera support for ZED