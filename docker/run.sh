#!/usr/bin/env bash

IMAGE=rvizweb

docker run -it --privileged --rm --network="host" "${IMAGE}"
