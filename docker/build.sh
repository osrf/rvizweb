#!/usr/bin/env bash

IMAGE=rvizweb

pushd "$( dirname "${BASH_SOURCE[0]}" )"
docker build -t ${IMAGE} "$@" .
popd

