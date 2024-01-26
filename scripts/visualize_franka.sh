#!/bin/bash

docker build -t urdf_creation ./.docker

echo 

docker run -it -u 1001 \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
    -v $(pwd):/workspaces/src/franka_description \
    -w /workspaces/src/franka_description \
    urdf_creation \
    .docker/visualize_franka.entrypoint.sh $*