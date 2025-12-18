#!/bin/bash
CONTAINER_NAME="advantech_vision"

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    docker exec -it -w /advantech_ws $CONTAINER_NAME bash -c "source /advantech_ws/entrypoint.sh && ros2 run orbbec_camera orbbec_camera_node --ros-args \
    -p enable_color:=true \
    -p enable_depth:=true \
    -r __ns:=/camera"
else
    echo "Error: Container '$CONTAINER_NAME' is not running."
    echo "Run ./build.sh first to start the container."
    exit 1
fi
##ros2 launch orbbec_camera gemini_330_series.launch.py
