#!/usr/bin/env bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
[ -f "/ros_ws/install/setup.bash" ] && source "/ros_ws/install/setup.bash"
# source "/opt/ACADOtoolkit/build/acado_env.sh"
# export PATH=$HOME/conda/bin:$HOME/conda/condabin:$PATH

for x in /opt/*; do
    if [[ -e "$x/.env.sh" ]]; then
	source "$x/.env.sh"
    fi
done

cd /ros_ws

echo 'irdl-tutorial-image startup completed.'
exec "$@"
