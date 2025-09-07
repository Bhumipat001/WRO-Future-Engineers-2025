#!/bin/bash

set -e

cleanup() {
    echo "Stopping background scripts..."
    for pid in "$IMU_PID" "$LIDAR_PID" "$DETECTION_PID" "$KEYMON_PID"; do
        if [ -n "$pid" ]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    exit 0
}

trap cleanup SIGINT

python3 imu.py &
IMU_PID=$!

python3 lidar.py &
LIDAR_PID=$!

python3 OpenChallenge.py &
DETECTION_PID=$!

echo "Press 'q' to stop all background scripts or Ctrl+C to interrupt."

(
    while true; do
        if read -r -n1 -s key < /dev/tty; then
            if [ "$key" = "q" ]; then
                echo "\n'q' pressed — stopping background scripts..."
                cleanup
            fi
        else
            sleep 0.2
        fi
    done
) &
KEYMON_PID=$!

trap cleanup SIGINT SIGTERM EXIT

wait $IMU_PID $LIDAR_PID $DETECTION_PID $KEYMON_PID