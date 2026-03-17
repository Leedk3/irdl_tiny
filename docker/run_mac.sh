#!/bin/bash

# check if inside container
tput setaf 1
if [ -n "$DOCKER_MACHINE_NAME" ]; then
  >&2 echo "Error: You probably are already inside a docker container!"
  tput sgr 0
  exit 1
elif [ ! -e /var/run/docker.sock ]; then
  >&2 echo "Error: Either docker is not installed or you are already inside a docker container!"
  tput sgr 0
  exit 1
fi
tput sgr 0

# directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
irdl_tiny_DIR="$(readlink -f "${SCRIPT_DIR}/../")"

# default config
CONTAINER_NAME="irdl-container-mac"

IMAGE_NAME="irdl-tiny-image"
IMAGE_TAG="mac"
IMAGE_REGISTRY=""

# read arguments
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -m|--mount)
    MOUNT_DIRS+=("$2")
    shift
    shift
    ;;
    -n|--name)
    CONTAINER_NAME="$2"
    shift
    shift
    ;;
    --tag)
    IMAGE_TAG="$2"
    shift
    shift
    ;;
    -h|--help)
    SHOW_HELP=1
    break
    ;;
    *)
    if [[ $1 == -* ]]; then
      echo "Invalid argument '$1'."
      SHOW_HELP=1
      break
    else
      POSITIONAL+=("$1")
      shift
    fi
    ;;
  esac
done

# pass either all positional arguments or the shell to docker
if [ ${#POSITIONAL[@]} -eq 0 ]; then
  ARGS=/bin/bash
else
  ARGS="${POSITIONAL[@]}"
fi

# show help
if [ "$SHOW_HELP" = 1 ]; then
  echo "Usage: ./run_mac.sh [-m|--mount DIR] [-n|--name NAME] [--tag TAG] [-h|--help] [ENTRYPOINT]"
  echo ""
  echo "Options:"
  echo " * -m|--mount DIR: Mount additional directory"
  echo " * -n|--name NAME: Docker container name"
  echo " * --tag TAG:      Image tag (default: mac)"
  echo " * -h|--help:      Show this message"
  echo ""
  echo "NOTE: For GUI (RViz2), install XQuartz and run: xhost +localhost"
  echo ""
  exit 1
fi

HOSTNAME=$(whoami)

# check for V4L2 devices (webcams etc.)
V4L2_DEVICES=""
for i in {0..9}; do
  if [ -a "/dev/video$i" ]; then
    V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
  fi
done

# docker arguments — no GPU flags for Mac
DOCKER_ARGS=(
  -v "${irdl_tiny_DIR}/../../":"/ros_ws/":z

  # no --runtime nvidia / --gpus on Mac
  --network host

  # X11 forwarding via TCP to XQuartz on the Mac host.
  #
  # Prerequisites (run once on your Mac):
  #   1. brew install --cask xquartz
  #   2. Open XQuartz → Preferences → Security → enable "Allow connections from network clients"
  #   3. Restart XQuartz, then run: xhost +localhost
  #
  # The container cannot access /tmp/.X11-unix because Docker Desktop runs
  # inside a Linux VM. TCP-based display (host.docker.internal:0) is used instead.
  -e DISPLAY=host.docker.internal:0

  # Force Mesa software rendering — XQuartz on Mac does not support GLX hardware acceleration
  -e LIBGL_ALWAYS_SOFTWARE=1
  -e MESA_GL_VERSION_OVERRIDE=3.3
  -e LIBGL_ALWAYS_INDIRECT=1
  -e OGRE_RTT_MODE=Copy
  -e QT_X11_NO_MITSHM=1

  # ROS2 DDS settings
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  -e DDS_INTERFACE="${DDS_INTERFACE}"
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}"
  -e CYCLONEDDS_URI="/ros_ws/src/irdl_tiny/.cyclonedds.xml"

  --add-host "${CONTAINER_NAME}:127.0.1.1"
  --name "${CONTAINER_NAME}"
  -h "${CONTAINER_NAME}"
  -e CONTAINER_NAME="${CONTAINER_NAME}"
  -e DOCKER_MACHINE_NAME="${CONTAINER_NAME}"

  -w /ros_ws
  -it
  --ipc=host
)

# user mapping: use username (not UID) so container /etc/passwd entry is found
DOCKER_ARGS+=(
  --user "$(whoami)"
)

# bag directory (create if it doesn't exist)
BAG_DIR="${HOME}/ros_ws/bag"
mkdir -p "${BAG_DIR}"
DOCKER_ARGS+=(-v "${BAG_DIR}:/bag:rw")

# mount additional directories
for dir in "${MOUNT_DIRS[@]}"; do
  DOCKER_ARGS+=(-v "${dir}:${dir}")
done

# run container
if docker ps -a --format '{{.Names}}' | grep -w $CONTAINER_NAME &> /dev/null; then
  if docker ps -a --format '{{.Status}}' | grep -E 'Exited|Created' &> /dev/null; then
    echo "Container exists but is stopped. Starting and attaching to ${CONTAINER_NAME}"
    docker start $CONTAINER_NAME
    docker exec -w "/ros_ws" -it $CONTAINER_NAME bash --init-file /tmp/etri_env.sh
  elif docker ps -a --format '{{.Status}}' | grep -E 'Up' &> /dev/null; then
    echo "Container is already running. Attaching to ${CONTAINER_NAME}"
    docker exec -w "/ros_ws" -it $CONTAINER_NAME bash --init-file /tmp/etri_env.sh
  fi
else
  echo "Opening docker env (Mac, no GPU)..."
  docker run --privileged=true \
    $V4L2_DEVICES \
    "${DOCKER_ARGS[@]}" \
    "${IMAGE_REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}" \
    "${ARGS}" || exit
fi
