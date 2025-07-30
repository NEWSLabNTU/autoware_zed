# ZED Launch Package

This package contains launch files for starting ZED cameras and visualization tools.

## Launch Files

### Multi-Camera with Object Detection
Launches multiple ZED cameras with object detection enabled:
```bash
ros2 launch zed_launch multi_camera_od.launch.xml
```

With custom parameters:
```bash
ros2 launch zed_launch multi_camera_od.launch.xml \
  cam_names:='[zed_front,zed_back]' \
  cam_models:='[zedxm,zedxm]' \
  cam_serials:='[59072113,50499972]'
```

### Visualization
Launches RViz2 with ZED camera visualization:
```bash
ros2 launch zed_launch visualization.launch.xml
```

With custom parameters:
```bash
ros2 launch zed_launch visualization.launch.xml \
  camera_name:='zed_front' \
  camera_model:='zedxm'
```

## Configuration Files

- `od_override.yaml`: Object detection configuration overrides (used by both single and multi-camera setups)