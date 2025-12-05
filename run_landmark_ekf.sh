#!/usr/bin/env bash
set -e

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

PIDS=()

echo "Starting Gazebo..."
ros2 launch prob_rob_labs turtlebot3_among_landmarks_launch.py &
PIDS+=($!)
sleep 6

echo "Starting Ground Truth publisher..."
ros2 launch prob_rob_labs ground_truth_publisher_launch.py &
PIDS+=($!)
sleep 2

echo "Starting EKF Localization..."
ros2 launch prob_rob_labs landmark_ekf_localization_launch.py &
PIDS+=($!)
sleep 2

echo "Starting EKF Error node..."
ros2 launch prob_rob_labs landmark_ekf_error_launch.py &
PIDS+=($!)
sleep 1

echo "Opening rqt_robot_steering..."
ros2 run rqt_robot_steering rqt_robot_steering &
PIDS+=($!)

echo "Opening rqt_plot..."
ros2 run rqt_plot rqt_plot &
PIDS+=($!)

echo "Everything launched!"
echo "Press Ctrl+C to stop everything."

cleanup() {
  echo "Shutting down all EKF lab processes..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait || true
  exit 0
}

trap cleanup SIGINT SIGTERM
wait
