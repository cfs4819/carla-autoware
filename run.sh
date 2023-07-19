#!/bin/bash

# Default settings
USER_ID="$(id -u)"
XAUTH=$HOME/.Xauthority

RUNTIME="--gpus all --runtime=nvidia"
# DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
# if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
#     RUNTIME="--gpus all"
# else
#     RUNTIME="--runtime=nvidia"
# fi

docker run \
    -it --rm \
    --name="autoware-$USER" \
    --volume=$(pwd)/autoware-contents:/home/autoware/autoware-contents:rw \
    --volume=/tmp/fuzzerdata/chenpansong:/tmp/fuzzerdata:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --net=host \
    --entrypoint=/bin/bash \
    $RUNTIME \
    carla-autoware:improved  \
    # Town05 '-20.17801284790039,-94.92411804199219,2.299999952316284,0.0,0.0,179.9136962890625'

