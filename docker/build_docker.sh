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
if [[ "${TYPE}" = "jetson" ]] || [[ "${TYPE}" = "ros2" ]]; then
  IMAGE_NAME="irdl-tiny-image:jetson"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/jetson.Dockerfile"
elif [[ "${TYPE}" = "x86" ]]; then
  IMAGE_NAME="irdl-tiny-image:${TAG}"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/x86.Dockerfile"
elif [[ "${TYPE}" = "mac" ]]; then
  IMAGE_NAME="irdl-tiny-image:${TAG}"
  DOCKERFILE="${SCRIPT_DIR}/dockerfiles/mac.Dockerfile"
else
  echo "Invalid Docker image type: ${TYPE}"
  echo "Usage: ./build_docker.sh [x86|jetson|mac] [DOCKER_ARGS [...]]"
  exit 1
fi

# CUDA build args — only for x86/jetson (not needed on Mac)
CUDA_BUILD_ARGS=()
if [[ "${TYPE}" != "mac" ]]; then
  CUDA_BUILD_ARGS=(
    --build-arg CUDA_PACKAGES="cuda-toolkit*"
    --build-arg CUDA_ARCH_LIST="87"
  )
fi

docker build \
  "${IRDL_DIR}" \
  --network=host \
  -t "${IMAGE_NAME}" \
  -f "${DOCKERFILE}" \
  --build-arg TAG="${TAG}" \
  --build-arg UID=$(id -u) \
  --build-arg USER=$(whoami) \
  "${CUDA_BUILD_ARGS[@]}" \
  ${DOCKER_ARGS[@]}
