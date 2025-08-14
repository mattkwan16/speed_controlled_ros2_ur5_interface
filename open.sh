#!/bin/bash

echo "Open http://localhost:6081/ to continue"
# TODO: replace mkwan with non-hardcoded
docker run --rm -it -p 6081:80 -p 50000-50020:50000-50020 \
-v "$(pwd)/docker/entrypoint.sh:/entrypoint.sh" \
-v "$(pwd)/:$(pwd)/" \
--entrypoint "/bin/bash" \
--security-opt seccomp=unconfined --shm-size=512m \
--net ursim_net --ip 192.168.56.200 \
--workdir "$(pwd)" \
--name ros2 mattkwan/speed_controlled_ros2_ur5_interface:latest \
-c "source /entrypoint.sh && /bin/bash"
