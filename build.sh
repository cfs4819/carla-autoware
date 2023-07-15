#!/bin/sh
docker build -t carla-autoware:improved -f Dockerfile . "$@"

