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
IRDL_DIR="$(readlink -f "${SCRIPT_DIR}/../")"

# type
if [[ "${TYPE}" = "ros" ]] || [[ "${TYPE}" = "ros2" ]]; then
  # config
  IMAGE_NAME="irdl-tutorial-image:ros2"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/ros.Dockerfile"
elif [[ "${TYPE}" = "x86" ]]; then
  # config
  IMAGE_NAME="irdl-tutorial-image:${TAG}"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/x86.Dockerfile"
else
  echo "Invalid Docker image type: ${TYPE}"
  exit 1
fi

docker build \
  "${IRDL_DIR}" \
  --network=host \
  -t "${IMAGE_NAME}" \
  -f "${DOCKERFILE}" \
  --build-arg TAG="${TAG}" \
  --build-arg UID=$(id -u) \
  --build-arg USER=$(whoami) \
  --build-arg CUDA_PACKAGES="cuda-toolkit*" \
  --build-arg CUDA_ARCH_LIST="87" \
  ${DOCKER_ARGS[@]}
