#!/bin/bash

# read arguments
ARGS=("$@")

if [[ $# -ge 1 ]]; then
  TYPE=${ARGS[0]}
  TAG=${ARGS[0]}
  DOCKER_ARGS=${ARGS[*]:1}
else
  echo "Usage: ./build_docker.sh TAG [DOCKER_ARGS [...]]"
  exit 1
fi

# directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
AI3CT="$(readlink -f "${SCRIPT_DIR}/../")"

# type
if [[ "${TYPE}" = "ros" ]] || [[ "${TYPE}" = "ros2" ]]; then
  # config
  IMAGE_NAME="ai3ct-ros:ros2"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/ros.Dockerfile"
elif [[ "${TYPE}" = "mac" ]]; then
  # config
  IMAGE_NAME="ai3ct-ros:${TAG}"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/mac.Dockerfile"
elif [[ "${TYPE}" = "jetson" ]]; then
  # config
  IMAGE_NAME="ai3ct-ros:ros2"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/jetson.Dockerfile"
elif [[ "${TYPE}" = "deploy" ]]; then
  # config
  IMAGE_NAME="ai3ct-ros:deploy"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/deploy.Dockerfile"
else
  echo "Invalid Docker image type: ${TYPE}"
  exit 1
fi

docker build \
  "${AI3CT}" \
  -t "${IMAGE_NAME}" \
  -f "${DOCKERFILE}" \
  --build-arg TAG="${TAG}" \
  --build-arg HOST_USERNAME="${USER}" \
  ${DOCKER_ARGS[@]}
