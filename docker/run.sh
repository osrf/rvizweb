#!/usr/bin/env bash

IMAGE=rvizweb

docker run -d --privileged --rm --name "sausy/rvizweb" -t "amd64" --network="host" "${IMAGE}"
