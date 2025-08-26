#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
exec $SCRIPT_DIR/util/setup_bridge.sh &
ros2 service call /world/default/set_pose ros_gz_interfaces/srv/SetEntityPose "{ entity: { name: 'block1', type: 2 }, pose: { position: { x: 0.325, y: 0.58, z: 0.90 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }"