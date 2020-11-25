#!/usr/bin/env bash

pushd "$( dirname "${BASH_SOURCE[0]}" )"
docker build -t "sausy/rvizweb:melodic" .
popd

