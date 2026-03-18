#!/bin/bash

# Start ArduPilot in background
arducopter --model quad --speedup 1 &
ARDUPILOT_PID=$!

# Wait for ArduPilot to start
sleep 3

# Start mavros in background
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://127.0.0.1:5760 -p gcs_url:=udp://@ &
MAVROS_PID=$!

# Wait for mavros to connect
sleep 40

# Set stream rates
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 1, message_rate: 10, on_off: true}"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 2, message_rate: 10, on_off: true}"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 6, message_rate: 10, on_off: true}"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 10, message_rate: 10, on_off: true}"

echo "ArduPilot and mavros started with stream rates configured!"
echo "Press Ctrl+C to stop everything"

# Wait for user to stop
wait
