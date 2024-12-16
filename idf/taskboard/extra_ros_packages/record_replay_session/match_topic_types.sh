#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash

# Make a temporary directory to store the topic types
mkdir -p /tmp/ros2_topic_types

# Get this script path
script_path=$(realpath $0)
script_path=$(dirname $script_path)

# Copy the topic types to the temporary directory
cp -r $script_path/../robothon_taskboard_msgs /tmp/ros2_topic_types/robothon_taskboard_msgs

# Build the topic types
cd /tmp/ros2_topic_types
colcon build --packages-up-to robothon_taskboard_msgs

# Source the workspace
source /tmp/ros2_topic_types/install/setup.bash

# Get all available topics
topics=$(ros2 topic list)

# Declare an array to store the process IDs of all subscribers
declare -a pids=()

# For each topic, start a subscriber in the background
for topic in $topics; do
    echo "Registering type for: $topic"
    # Run ros2 topic echo in background
    ros2 topic echo "$topic" > /dev/null &
    # Store the PID of the last background process
    pids+=($!)
done

# Wait 2 seconds to gather some messages
sleep 2

# Kill all subscriber processes
echo "Terminating..."
kill "${pids[@]}"
