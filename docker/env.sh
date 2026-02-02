#!/usr/bin/env bash
#
# Copyright 2017 - 2018 Ternaris
# SPDX-License-Identifier: Apache 2.0

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/etri_ws/install/setup.bash"
# source "/opt/ACADOtoolkit/build/acado_env.sh"
export PATH=$HOME/conda/bin:$HOME/conda/condabin:$PATH

for x in /opt/*; do
    if [[ -e "$x/.env.sh" ]]; then
	source "$x/.env.sh"
    fi
done

cd /etri_ws

echo 'AI3CT startup completed.'
exec "$@"
