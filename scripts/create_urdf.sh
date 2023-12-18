#!/bin/bash

docker build -t urdf_creation ./.docker

echo 

docker run -u 1001 \
    -v $(pwd):/workspaces/src/franka_description \
    -w /workspaces/src/franka_description \
    urdf_creation \
    .docker/create_urdf.entrypoint.sh $*