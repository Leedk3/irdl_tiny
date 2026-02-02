#!/bin/bash

# read arguments
ARGS=("$@")

if [[ $# -ge 2 ]]; then
  IMAGE_NAME=${ARGS[0]}
  TAG=${ARGS[1]}
  DOCKER_ARGS=${ARGS[*]:2}
else
  echo "Usage: ./build_docker.sh IMAGE_NAME TAG [DOCKER_ARGS [...]]"
  exit 1
fi

# directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
TARGET_DIR="$(readlink -f "${SCRIPT_DIR}/../")"

# type
IMAGE_NAME="${IMAGE_NAME}"
DOCKERFILE="${SCRIPT_DIR}/dockerfiles/perception_base_image.Dockerfile"

echo "###### BUILDING DOCKER IMAGE... ######"
echo "==>             ${IMAGE_NAME}"
echo "##########################"

docker build \
  "${TARGET_DIR}" \
  -t "${IMAGE_NAME}" \
  -f "${DOCKERFILE}" \
  ${DOCKER_ARGS[@]}
