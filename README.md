# Advantech-ROS2-VLM-Container-on-NVIDIA-Jetson-Thor

## Overview

This package provides a containerized ROS2 Jazzy environment with CUDA 13.0, OpenCV 4.13, and PyTorch for NVIDIA Jetson Thor (JetPack 7.0). It includes the Orbbec camera SDK and Advantech vision applications with VLM integration, YOLO object detection, and RGB-D depth estimation.

> ⚠️ **Notice**
>
> Some packages, container images, model files, or SDK components required by this project are **not included in this repository**.
>  
> These files are provided **upon request** due to licensing, size, or distribution restrictions.
>  
> Please contact the maintainer for access.

## System Requirements

### Hardware Requirements
- NVIDIA Jetson AGX Thor Developer Kit
  - Jetson T5000 SoM with Blackwell GPU
  - 128GB LPDDR5X RAM
  - 1TB NVMe SSD (included with devkit)
  - 2070 FP4 TFLOPS AI performance
- Advantech CAM-5330 (Orbbec Gemini 336) RGB-D Camera
- USB 3.0 port for camera connection
- Display connected for visualization (HDMI 2.1 or DisplayPort 1.4a)

### Software Requirements
- JetPack 7.0 (L4T R38.2.0)
- Docker with NVIDIA Container Runtime
- Docker Compose v1.29.2 or above
- X11 display server
- Orbbec SDK and udev rules (on host)

### Pre-Installation Checks

```bash
# Verify NVIDIA runtime is available
docker info | grep -i nvidia

# Verify JetPack version
cat /etc/nv_tegra_release

# Verify Docker Compose
docker-compose version

# Check Jetson power mode
sudo nvpmodel -q
```

## Host Prerequisites: Orbbec SDK Setup

**Important:** For reliable camera detection and streaming, install the Orbbec SDK and udev rules on the host system before running the container.

### Official Documentation

- **OrbbecSDK ROS2 Wrapper**: https://github.com/orbbec/OrbbecSDK_ROS2
- **Gemini 330 Series Documentation**: https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation

### Install Orbbec SDK on Host

```bash
# Create workspace
mkdir -p ~/orbbec_sdk
cd ~/orbbec_sdk

# Clone OrbbecSDK ROS2 wrapper
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Install udev rules (required for USB device permissions)
cd ~/orbbec_sdk/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Verify Camera on Host

```bash
# Check USB device is detected
lsusb | grep 2bc5

# Expected output:
# Bus 002 Device 003: ID 2bc5:0816 Orbbec 3D Technology International

# Check device permissions
ls -la /dev/bus/usb/002/
```

### Why Host SDK is Required

- Provides correct udev rules for USB device permissions
- Ensures proper USB port path resolution
- Enables reliable device enumeration
- Fixes common "uvc_open failed" errors

## Directory Structure

```
advantech-vision/
├── build.sh                           # Build and start container
├── run.sh                             # Launch camera node
├── docker-compose.yml                 # Container configuration
├── advantech_ros2_vision_v1.0.tar.gz  # Container image                      # Provided upon request
├── README.md
└── ros2/                              # ROS2 workspace (mounted)             # Provided upon request
    ├── entrypoint.sh                  # Environment setup script
    ├── cache                          # cached vlm and yolo models directory
    ├── src/                           # Source packages
    │   ├── advantech_vision/          # Vision application
    │   └── OrbbecSDK_ROS2/            # Orbbec camera driver
    ├── build/                         # Build artifacts
    ├── install/                       # Installed packages
    └── log/                           # Build logs
```

## First Time Setup

### 1. Prepare Files

Place all files in the `advantech-vision` directory and make scripts executable:

```bash
chmod +x build.sh run.sh ros2/entrypoint.sh
```

### 2. Enable X11 Display Access

**Important:** Run this command on the host before starting the container:

```bash
xhost +local:root
```

To make this permanent, add to `~/.bashrc`:

```bash
echo 'xhost +local:root > /dev/null 2>&1' >> ~/.bashrc
```

### 3. Load and Start Container

```bash
./build.sh
```

This will:
- Extract the image from tar.gz (first time only)
- Start the container with GPU access
- Enter an interactive shell with ROS2 environment sourced

### 4. Build ROS2 Packages (First Time or After Code Changes)

Inside the container, run the following if you started the container using **docker exec -it advantech_vision bash**:

```bash
# Activate Python virtual environment (inside container)
source /opt/venv/bin/activate

# Clean environment variables (prevents stale path warnings)
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Source ROS2 base
source /opt/ros/jazzy/install/setup.bash

# Build all packages
colcon build --cmake-args -Wno-dev

# Source the workspace
source install/setup.bash

# Set library path
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
```

### Quick Build Command (After Initial Setup)

For subsequent builds when environment is already set up:

```bash
colcon build --cmake-args -Wno-dev && source install/setup.bash
```

### Build Specific Package Only

```bash
colcon build --packages-select advantech_vision && source install/setup.bash
colcon build --packages-select orbbec_camera && source install/setup.bash
```

## Running the Application

### Terminal 1: Start Vision Application(using ./build.sh)

```bash
ros2 run advantech_vision advantech_vlm_camera_node --ros-args \
    -p yolo_model:=/advantech_ws/yolo11n-seg.pt \
    -p cache_dir:=/advantech_ws/cache \
    -p use_tensorrt:=true
```


### Terminal 2: Start Camera Node

```bash
ros2 run orbbec_camera orbbec_camera_node --ros-args \
    -p enable_color:=true \
    -p enable_depth:=true \
    -r __ns:=/camera
```
Or use the helper script:

```bash
./run.sh
```

### Terminal 3: Verify Camera Streams(optional)
open new terminal and enter the same container using the following command refer [Build ROS2 Packages](#Build-ROS2-Packages) for environment sourcing

```
docker exec -it advantech_vision bash
```

```bash
# Check topics are publishing
ros2 topic list | grep camera

# Verify frame rates
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

## Interactive VLM Query System

The vision application includes an interactive VLM (Vision Language Model) query system that allows natural language questions about detected objects and the scene.

### Activating Query Mode

1. Press **`F`** key to activate the query input window
2. Type your question in the prompt window
3. Press **Enter** to submit the query
4. View the VLM response in the info panel

### Query Input Limits

| Parameter | Limit |
|-----------|-------|
| Maximum characters | 256 |
| Recommended length | 10-50 characters |
| Response time | 1-3 seconds (depending on query complexity) |

### Example Questions You Can Ask

#### Object Location Queries
- "Where is the keyboard?"
- "Find the person"
- "Locate the chair"
- "Where is the TV?"

#### Spatial Relationship Queries
- "What is near the keyboard?"
- "What objects are on the left?"
- "What is in the center of the frame?"
- "What is closest to the camera?"

#### Object Description Queries
- "Describe the scene"
- "What objects do you see?"
- "How many chairs are there?"
- "What color is the keyboard?"

#### Distance Queries
- "How far is the chair?"
- "What is the distance to the TV?"
- "Which object is nearest?"

### VLM Response Format

The system provides responses with:
- **Object name**: Detected object class
- **Position**: left/center/right + top/middle/bottom
- **Distance**: In meters (when depth data available)
- **Nearby objects**: Other objects within 0.5m

Example response:
```
keyboard: left-bottom, near (0.65m)
Nearby: mouse, monitor
```

### VLM Limitations

| Limitation | Description |
|------------|-------------|
| Detection dependency | Can only answer about YOLO-detected objects (80 COCO classes) |
| Depth range | Accurate depth: 0.5m - 10m |
| Query cooldown | Minimum 2 seconds between queries |
| Language | English only |
| Complex reasoning | Limited multi-step reasoning capability |
| Occlusion | Cannot detect fully occluded objects |

### Supported Object Classes

The YOLO model detects 80 COCO classes including:
- **People**: person
- **Vehicles**: car, bicycle, motorcycle, bus, truck
- **Furniture**: chair, couch, bed, dining table
- **Electronics**: tv, laptop, cell phone, keyboard, mouse
- **Kitchen**: bottle, cup, fork, knife, spoon, bowl
- **Animals**: cat, dog, bird, horse
- **Others**: book, clock, vase, scissors, teddy bear

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `yolo_model` | yolo11n-seg.pt | YOLO model file path |
| `yolo_conf_threshold` | 0.5 | Detection confidence threshold |
| `yolo_iou_threshold` | 0.4 | IoU threshold for NMS |
| `use_tensorrt` | true | Enable TensorRT acceleration |
| `enable_vlm` | true | Enable Visual Language Model |
| `vlm_model` | llava-hf/llava-1.5-7b-hf | VLM model name |
| `vlm_quantize_bits` | 4 | VLM quantization (4 or 8) |
| `cache_dir` | /tmp/advantech_cache | Model cache directory |

## Keyboard Controls (Vision Application)

| Key | Action |
|-----|--------|
| `F` | Activate query input (VLM prompt) |
| `C` | Clear target tracking |
| `S` | Save snapshot |
| `V` | Toggle VLM on/off |
| `R` | Reset tracking |
| `Q` | Quit application |

## Environment Variables

Set these before running applications:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/install/setup.bash
source /advantech_ws/install/setup.bash
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb
export OPENCV_VIDEOIO_PRIORITY_MSMF=0
export DISPLAY=:0
```

## GPU Verification

```bash
# Check GPU status
nvidia-smi

# Verify CUDA availability in Python
python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"

# Check TensorRT
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

## Performance Optimization

### Jetson Power Mode

```bash
# Set maximum performance mode
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks

# Monitor power and temperature
sudo tegrastats
```

### GPU Memory Monitoring

```bash
# Real-time GPU monitoring
watch -n 1 nvidia-smi
```

## Monitor ROS2 Topics

```bash
# List all topics
ros2 topic list

# View topic info
ros2 topic info /camera/color/image_raw

# Echo topic data (single message)
ros2 topic echo /camera/color/image_raw --once

# Check publishing rate
ros2 topic hz /camera/color/image_raw
```

## Custom Python Packages

The Python virtual environment `/opt/venv` is located **inside the container**. To install additional packages:

```bash
# Enter container
docker exec -it advantech_vision /bin/bash

# Activate virtual environment (inside container)
source /opt/venv/bin/activate

# Install package (venv isolates packages, no special flags needed)
pip install your-package

# For packages from standard PyPI (not Jetson index)
pip install --index-url https://pypi.org/simple your-package
```

**Note:** The virtual environment is baked into the container image. Packages installed this way persist until the container is removed/recreated.

## Volume Persistence

The following data persists between container restarts:

| Volume/Mount | Path | Contents |
|--------------|------|----------|
| `./ros2` | `/advantech_ws` | ROS2 workspace, source code, builds |
| `/dev` | `/dev` | Device access |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display socket |

## FAQs

### X11 Display Errors

**Error:** `Authorization required, but no authorization protocol specified`

**Solution:** Run on host machine:
```bash
xhost +local:root
```

### Camera Not Detected

**Error:** `uvc_open failed: Return Code: -6`

**Solution:**
1. Ensure Orbbec SDK udev rules are installed on host:
   ```bash
   cd ~/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
   sudo bash install_udev_rules.sh
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
2. Check camera is connected:
   ```bash
   lsusb | grep 2bc5
   ```
3. Verify device permissions:
   ```bash
   ls -la /dev/bus/usb/
   ```
4. Restart container with camera connected

### Missing Library Errors

**Error:** `cannot open shared object file: libbackward.so`

**Solution:**
```bash
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
```

### AMENT_PREFIX_PATH Warnings

**Warning:** `The path '/advantech_ws/install/...' doesn't exist`

**Solution:** Clean environment before building:
```bash
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /opt/ros/jazzy/install/setup.bash
colcon build
```

### Camera Topics Not Publishing

**Issue:** Topics exist but no data

**Solution:** Verify camera is streaming:
```bash
ros2 run orbbec_camera list_devices_node
```

If device is listed but not streaming, restart the camera node with explicit parameters:
```bash
ros2 run orbbec_camera orbbec_camera_node --ros-args \
    -p enable_color:=true \
    -p enable_depth:=true \
    -r __ns:=/camera
```

### VLM Query Returns "No Object Detected"

**Issue:** VLM says no object detected even when objects are visible

**Solutions:**
1. Ensure YOLO is detecting objects (check bounding boxes on display)
2. Lower detection confidence threshold:
   ```bash
   ros2 run advantech_vision advantech_vlm_camera_node --ros-args \
       -p yolo_conf_threshold:=0.3
   ```
3. Check if object class is in COCO dataset (80 classes supported)

### Depth Data Unavailable

**Issue:** Objects show position but no distance

**Possible causes:**
- Object too close (< 0.3m) or too far (> 10m)
- Reflective or transparent surfaces
- IR interference from sunlight

**Solution:** Move object to optimal range (0.5m - 5m)

### Application Freezes on Quit

**Issue:** Window freezes when pressing 'Q'

**Solution:** Use Ctrl+C in terminal to force stop, then:
```bash
pkill -f orbbec_camera
pkill -f advantech_vlm
```

### TensorRT Engine Warnings

**Warning:** `Using an engine plan file across different models of devices`

This warning is informational and can be ignored. The engine will still function correctly.

### Python Package Installation Errors

**Error:** `Could not find a version that satisfies the requirement`

**Solution:** Specify standard PyPI index:
```bash
pip install --index-url https://pypi.org/simple package-name
```

## Stop Container

```bash
docker compose down
```

## Rebuild Container Image

If you need to rebuild the Docker image:

```bash
docker compose down
docker rmi advantech_ros2_vision:v1.0
./build.sh
```

## Environment Details

| Component | Version |
|-----------|---------|
| Platform | NVIDIA Jetson AGX Thor |
| JetPack | 7.0 |
| ROS | Jazzy |
| CUDA | 13.0 |
| cuDNN | 9.x |
| OpenCV | 4.13.0 |
| Python | 3.12 |
| PyTorch | 2.x |
| TensorRT | 10.x |
| Orbbec SDK | 1.10.x |

## References

- **OrbbecSDK ROS2**: https://github.com/orbbec/OrbbecSDK_ROS2
- **Orbbec Gemini 330 Series Docs**: https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation
- **NVIDIA Jetson Thor**: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/

## Maintainer
Benson Chen (benson.chen@advantech.com.tw)
Samir Singh (samir.singh@advantech.com)
Advantech Corporation
