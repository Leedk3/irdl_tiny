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
AI3CT_DIR="$(readlink -f "${SCRIPT_DIR}/../")"

# default config
GPU_NUM="all"
CONTAINER_NAME="ai3ct-${GPU_NUM}"

ROS_MASTER_URI="${ROS_MASTER_URI}"
ROS_IP="${ROS_IP}"

# TEST_LOC="ETRI" 
# TESTSITE_LAT_ORIGIN="36.381365" 
# TESTSITE_LON_ORIGIN="127.364937" 


# TEST_LOC="INCHEON_AIRPORT" 
# TESTSITE_LAT_ORIGIN="37.463333" 
# TESTSITE_LON_ORIGIN="126.440002" 

TEST_LOC="SEOUL" 
TESTSITE_LAT_ORIGIN="37.5383502" 
TESTSITE_LON_ORIGIN="126.9300699" 


IMAGE_REGISTRY=""
IMAGE_NAME="ai3ct-ros"
IMAGE_TAG="ros2" #"latest"
USER=1
IMAGE_REGISTRY=""
# read arguments
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -m|--mount)
    MOUNT_DIRS+=("$2")
    shift # past argument
    shift # past value
    ;;
    -n|--name)
    CONTAINER_NAME="$2"
    shift # past argument
    shift # past value
    ;;
    --tag)
    IMAGE_TAG="$2"
    shift # past argument
    shift # past value
    ;;
    -h|--help)
    SHOW_HELP=1
    break
    ;;
    *)    # invalid option
    if [[ $1 == -* ]]; then
      echo "Invalid argument '$1'."
      SHOW_HELP=1
      break
    else
      POSITIONAL+=("$1")
      shift # past argument
    fi
    ;;
  esac
done

# pass either all positional arguments or the shell to docker
if [ ${#POSITIONAL[@]} -eq 0 ]; then
  ARGS=${SHELL}
else
  ARGS="${POSITIONAL[@]}"
fi

# show help
if [ "$SHOW_HELP" = 1 ]; then
  echo "Usage: ./run_docker.bash [--deps] [--home] [--local] [-m|--mount DIR [-m|--mount DIR ...]] [--tag TAG] [-h|--help] [ENTRYPOINT]"
  echo ""
  echo "If no ENTRYPOINT is given, your shell ($SHELL) is used."
  echo ""
  echo "Options:"
  echo " * --user:         Use current user and group within the container and mount the home directory."
  echo " * --local:        Use local image instead of image from GitHub registry --> currently no uploaded github. it is private."
  echo " * -m|--mount DIR: Mount directory DIR"
  echo " * -n|--name NAME: Docker container name (default: ai3ct)"
  echo " * --tag TAG:      Image tag (default: latest version)"
  echo " * -h|--help:      Show this message"
  echo ""
  exit 1
fi
HOSTNAME=$(whoami)

# docker arguments
DOCKER_ARGS=(
  -v "${AI3CT_DIR}/../../":"/etri_ws/":z
  # -v "/home/leedk/test/build":"/etri_ws/build":z
  # -v "/home/leedk/test/log":"/etri_ws/log":z
  # -v "/home/leedk/test/install":"/etri_ws/install":z

  --runtime nvidia
  --network host
  # xserver access for visualization in test scripts
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -e DISPLAY="${DISPLAY}"
  -v /usr/local/cuda:/usr/local/cuda
  -v /home/${HOSTNAME}/.vscode/extensions:/home/${HOSTNAME}/.vscode/extensions

  # container name
  --name "${CONTAINER_NAME}"
  -h "${CONTAINER_NAME}"
  -e CONTAINER_NAME="${CONTAINER_NAME}"
  -e DOCKER_MACHINE_NAME="${CONTAINER_NAME}"
  -e ROS_MASTER_URI="${ROS_MASTER_URI}"
  -e ROS_IP="${ROS_IP}"
  -e TEST_LOC="${TEST_LOC}"
  -e TESTSITE_LAT_ORIGIN="${TESTSITE_LAT_ORIGIN}"
  -e TESTSITE_LON_ORIGIN="${TESTSITE_LON_ORIGIN}"
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/etri_ws/fastdds.xml
  -e LOG_BAG_LIST="/Ackermann/veh_state /LongiCont/speed_debug /Odometry/base /can_AccBrkSft /can_input_msgs /can_longi_debug /can_steer_debug /gnss1/aiding_status /gnss1/antenna_offset_correction /gnss1/fix /gnss1/fix_info /gnss1/odom /gnss1/time_ref /gnss2/aiding_status /gnss2/antenna_offset_correction /gnss2/fix /gnss2/fix_info /gnss2/odom /gnss2/time_ref /imu/data /imu/secondary/data /ins_localizer/odom /left/velodyne_points /mag /nav/aiding_summary /nav/dual_antenna_status /nav/filtered_imu/data /nav/heading /nav/odom /nav/relative_pos/odom /nav/status /ntrip_client/nmea/sentence /ouster/points /ouster/range_image /right/velodyne_points /tf"
  -e PATH="/home/leedk/.local/bin:$PATH"
  # misc
  -w /etri_ws  # set ros_ws directory as workspace
  -it  # run container in interactive mode
  # --rm  # automatically remove container when it exits
  --ipc=host
  # --ulimit nofile=1024  # makes forking processes faster, see https://github.com/docker/for-linux/issues/502
)

# dependency vs deploy image
if [ "${DEPS}" = 1 ]; then
  DOCKER_ARGS+=(-e PYTHONPATH="${AI3CT_DIR}:${PYTHONPATH}")
else
  DOCKER_ARGS+=(-e PYTHONPATH="${PYTHONPATH}")
fi

# environment variables
if [[ -n "${KITTI_PATH}" ]]; then
  DOCKER_ARGS+=(-e KITTI_PATH="${KITTI_PATH}")
fi
if [[ -n "${MODEL_PATH}" ]]; then
  DOCKER_ARGS+=(-e MODEL_PATH="${MODEL_PATH}")
fi

# user
if [ "${USER}" = 1 ]; then
  DOCKER_ARGS+=(
    -v /etc/passwd:/etc/passwd:ro
    -v /etc/group:/etc/group:ro
    --user "$(id -u):$(id -g)"
    # -v "${HOME}:${HOME}"
  )
else
  DOCKER_ARGS+=(
    --user 1000:1000
  )
fi

# my statement
DOCKER_ARGS+=(-v /home/${HOSTNAME}/Dataset:/etri_ws/etri_bag:rw)
DOCKER_ARGS+=(--device /dev/input/js0)
# DOCKER_ARGS+=(-v /home/usrg/Data/Dataset/3D_data/localize/dataset:/home/usrg/deepclr/data/original:rw)
# DOCKER_ARGS+=(-e KITTI_PATH="/home/usrg/Data/Dataset/3D_data/localize/dataset")
# DOCKER_ARGS+=(-e MODLE_PATH=/home/usrg/deepclr/models)

# mount directories
for dir in "${MOUNT_DIRS[@]}"; do
  DOCKER_ARGS+=(-v "${dir}:${dir}")
done

# run container
if docker ps -a --format '{{.Names}}' | grep -w $CONTAINER_NAME &> /dev/null; then
	if docker ps -a --format '{{.Status}}' | egrep 'Exited' &> /dev/null; then
		echo "Container is already running. Attach to ${CONTAINER_NAME}"
		docker start $CONTAINER_NAME 	
		docker exec -w "/etri_ws" -it $CONTAINER_NAME bash --init-file /tmp/etri_env.sh
	elif docker ps -a --format '{{.Status}}' | egrep 'Created' &> /dev/null; then
		echo "Container is already created. Start and attach to ${CONTAINER_NAME}"
		docker start $CONTAINER_NAME 	
		docker exec -w "/etri_ws" -it $CONTAINER_NAME bash --init-file /tmp/etri_env.sh  
	elif docker ps -a --format '{{.Status}}' | egrep 'Up' &> /dev/null; then
		echo "Docker is already running"
		docker exec -w "/etri_ws" -it $CONTAINER_NAME bash --init-file /tmp/etri_env.sh
	fi 
else
  echo "Opening docker env...."
  docker run --privileged=true --gpus "device=${GPU_NUM}" \
    "${DOCKER_ARGS[@]}" \
    "${IMAGE_REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}" \
    "${ARGS}" || exit 1
fi


# # run container
# docker run --gpus '"device=1"' \
#   "${DOCKER_ARGS[@]}" \
#   "${IMAGE_REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}" \
#   "${ARGS}" || exit 1

# # run container
# docker run --gpus '"device=0,1"' \
#   "${DOCKER_ARGS[@]}" \
#   "${IMAGE_REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}" \
#   "${ARGS}" || exit 1
