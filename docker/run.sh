#!/usr/bin/env bash

IMAGE=rvizweb

docker run -d --privileged --rm --name "${IMAGE}" --network="host" "${IMAGE}"
