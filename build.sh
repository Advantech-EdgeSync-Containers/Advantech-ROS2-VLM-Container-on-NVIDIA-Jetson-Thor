#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="advantech_vision"
IMAGE_NAME="advantech_ros2_vision:v1.0"
TAR_FILE="advantech_ros2_vision_v1.0.tar.gz"

cd "$SCRIPT_DIR"
chmod +x ros2/entrypoint.sh
echo "Stopping existing container if running..."
docker-compose down 2>/dev/null || true

if docker image inspect $IMAGE_NAME &>/dev/null; then
    echo "Image '$IMAGE_NAME' already exists. Skipping load."
else
    if [ -f "$TAR_FILE" ]; then
        echo "Loading image from $TAR_FILE..."
        gunzip -c "$TAR_FILE" | docker load
        echo "Image loaded successfully."
    else
        echo "Error: Neither image '$IMAGE_NAME' nor file '$TAR_FILE' found."
        echo "Please place '$TAR_FILE' in the same directory as this script."
        exit 1
    fi
fi

echo "Creating and starting container..."
docker-compose up -d
echo "Waiting for container to start..."
sleep 5

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container started successfully."
    echo "Entering container..."
    docker exec -it -w /advantech_ws $CONTAINER_NAME bash --rcfile /advantech_ws/entrypoint.sh
else
    echo "Error: Container failed to start."
    docker-compose logs
    exit 1
fi
