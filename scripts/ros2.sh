#!/usr/bin/env bash

docker run --rm -it -p 6081:80 -p 50000-50020:50000-50020 \
-v "$(pwd)/docker/entrypoint.sh:/entrypoint.sh" \
-v "$(pwd)/:$(pwd)/" \
--entrypoint "/bin/bash" \
--security-opt seccomp=unconfined --shm-size=512m \
--net ursim_net --ip 192.168.56.200 \
--workdir "$(pwd)" \
--name ros2 mattkwan/speed_controlled_ros2_ur5_interface:latest \
-c "/entrypoint.sh"

# To connect to the real robot or to the docket simulator, you need to run the following command:
# docker run --rm -d -p 6081:80 --security-opt seccomp=unconfined --shm-size=512m --net ursim_net --ip 192.168.56.200 --name ros2 pla10/ros2_ur5_interface

# If you want to mount a volume to the container to work with your code, you can use the following option:
# -v /HOST/PATH/my_code:/home/ubuntu/ros2_ws/src/my_package 
