#!/bin/bash
# Grant access to the X server
xhost +local:root

# Name for the container
CONTAINER_NAME=ros_noetic_container

# Launch ROS Noetic container with GUI support and set it up
docker run -d \
    --name $CONTAINER_NAME \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p 11311:11311 \
    --net=host \
    osrf/ros:noetic-desktop-full \
    bash -c "source /opt/ros/noetic/setup.bash && roscore"

# 
# Give some time for roscore to start up
sleep 5

# Get container IP
# CONTAINER_IP=$(docker inspect $CONTAINER_ID | grep IPAddress | cut -d '"' -f 4)
CONTAINER_IP="127.0.0.1" #$(docker inspect $CONTAINER_NAME | grep IPAddress | head -n 2 | cut -d '"' -f 4 | tr -d '\n\r')

echo "CONTAINER_IP is $CONTAINER_IP"

# Set up ROS environment variables on host
export ROS_MASTER_URI=http://$CONTAINER_IP:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

echo "ROS Noetic container is running with ROS_MASTER_URI set to $ROS_MASTER_URI"
echo "Your host ROS_IP is set to $ROS_IP"

echo "To run commands inside the container, use: docker exec -it $CONTAINER_NAME <command>"
