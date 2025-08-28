#!/bin/bash

# Specify the container name
CONTAINER_NAME="drims2"
# IMAGE_NAME="smentasti/drims2:2025"
IMAGE_NAME="drims-ros2"
DOCKER_IMAGE="drims-ros2"
#IMAGE_NAME="my_image"

# Grant X permissions
#xhost +si:localuser:$(whoami)
xhost +local:root
# Check if the container exists
# if docker ps -a | grep -q $CONTAINER_NAME; then
#     echo "Container $CONTAINER_NAME exists."
# 
#     # Check if the container is running
#     if [ "$(docker inspect -f {{.State.Running}} $CONTAINER_NAME)" == "true" ]; then
#         echo "Container $CONTAINER_NAME is running. Stopping it now..."
#         docker stop $CONTAINER_NAME
#         docker rm $CONTAINER_NAME
#     else
#         echo "Container $CONTAINER_NAME is not running."
#         docker rm $CONTAINER_NAME
#     fi
# else
#     echo "Container $CONTAINER_NAME does not exist."
# fi

if  [[ $1 ]]; then case "$1" in
    -h|--help|help)
        echo "Starts a docker container names '$CONTAINER_NAME' based on image '$DOCKER_IMAGE'"
        echo "Possible parameters:"
        echo "  --run:    runs the container"
        echo "  --build:  builds the image"
        echo "  --stop:   stops the container (if running)"
        echo "  --rm:     removes the container"
        exit 0
        ;;
    --build)
        BUILD_IMAGE=true
        RUN_CONTAINER=false
        ;;
    --run)
        BUILD_IMAGE=false
        RUN_CONTAINER=true
        ;;
    --stop)
        echo "Stopping container $CONTAINER_NAME"
        docker stop $CONTAINER_NAME >/dev/null 2> /dev/null 
        exit 0
        ;;
    --rm)
        echo "Removing container $CONTAINER_NAME"
        docker stop $CONTAINER_NAME >/dev/null 2> /dev/null 
        docker rm $CONTAINER_NAME >/dev/null 2> /dev/null 
        exit 0
        ;;
esac
fi

# Get the absolute path of this script
# Source: https://stackoverflow.com/questions/59895
THIS_SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR=$(realpath $THIS_SCRIPT_DIR/..)

# Ensure that base image is available
if [[ -z  "$(docker images -q osrf/ros:humble-desktop-full 2> /dev/null)"  ]]; then
    echo "Base image 'osrf/ros:humble-desktop-full' not found, pulling it from the hub..."
    docker pull osrf/ros:humble-desktop-full
fi

# Ensure that image is available
if [[ -z  "$(docker images -q $DOCKER_IMAGE 2> /dev/null)"  ]]; then
    echo "Image $DOCKER_IMAGE not found locally"
    BUILD_IMAGE=true
fi

if [ "$BUILD_IMAGE" = true ]; then
    if [ "$(docker ps -a | grep $CONTAINER_NAME)" ]; then
        docker rm $CONTAINER_NAME
    fi
    docker build -t $DOCKER_IMAGE . --build-arg base_image=ros2-venv
fi

if [ "$RUN_CONTAINER" = false ]; then
    exit 0;
fi

xhost +local:docker

# Check if container is already running
if [[ -n "$(docker ps -q -f name=$CONTAINER_NAME)" ]]; then
    echo "Container $CONTAINER_NAME is already running, attaching to it.."
    docker exec -it $CONTAINER_NAME /bin/bash
else
    # Check if container is stopped
    if [[ -n "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]]; then
        echo "Container $CONTAINER_NAME is stopped, starting it.."
        docker start $CONTAINER_NAME
        docker exec -it $CONTAINER_NAME /bin/bash
    else
        echo "Container $CONTAINER_NAME not found, creating and starting it.."
        docker run \
            -it  \
            --user drims \
            --privileged \
            -v /dev:/dev \
            -v /dev/bus/usb:/dev/bus/usb \
            --device=/dev/bus/usb \
            --device-cgroup-rule='c 189:* rmw' \
            -v /etc/udev/rules.d:/etc/udev/rules.d \
            --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
            --net=host \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="$(pwd)/drims_ws:/home/drims/drims_ws" \
            --volume="$(pwd)/bags:/home/drims/bags"  \
            --name drims2 \
            -w /home/drims \
            $IMAGE_NAME

    fi
fi
