#!/usr/bin/env bash
# Force-kill everything left over from an interrupted Go2 simulation run
# (gzserver/gzclient, roslaunch, spawners, rviz, junior_ctrl, BLE bridge).
# A stale gzserver keeps port 11345 busy, which makes the next launch hang or
# fail to spawn the robot.
#
# Usage: rosrun unitree_gazebo clean_sim.sh [max_attempts]

set -u

PATTERNS=(
    "gzserver"
    "gzclient"
    "roslaunch unitree_gazebo"
    "roslaunch unitree_guide"
    "junior_ctrl"
    "unitree_ble_joy"
    "controller_manager/spawner"
    "gazebo_ros/spawn_model"
    "odom_tf_publisher"
    "lidar_self_filter"
    "robot_state_publisher"
    "rviz"
)

max_attempts="${1:-5}"

for attempt in $(seq 1 "$max_attempts"); do
    pids=""
    for pat in "${PATTERNS[@]}"; do
        pids="$pids $(pgrep -f "$pat" 2>/dev/null || true)"
    done
    pids=$(echo "$pids" | tr ' ' '\n' | grep -E '^[0-9]+$' | grep -v "^$$\$" | sort -u)

    if [ -z "$pids" ]; then
        echo "clean_sim.sh: no matching processes remain (attempt $attempt)."
        exit 0
    fi

    echo "clean_sim.sh: attempt $attempt, killing PIDs: $(echo "$pids" | tr '\n' ' ')"
    echo "$pids" | xargs -r kill -9
    sleep 1
done

echo "clean_sim.sh: WARNING -- processes still running after $max_attempts attempts."
exit 1
