#!/bin/bash

ros2 service call /world/default/set_pose ros_gz_interfaces/srv/SetEntityPose "{ entity: { name: 'block1', type: 2 }, pose: { position: { x: 0.825, y: 0.58, z: 0.90 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }"