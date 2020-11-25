#!/usr/bin/env bash

IMAGE=rvizweb

docker run -d --privileged --rm --network="host" "${IMAGE}"
