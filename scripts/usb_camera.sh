#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."
root_path=$(pwd)

function start() {
    LOG="/apollo/data/log/usb_cam.out"
    case $1 in
        *)
        CMD="roslaunch usb_cam start_$1_leopard.launch"
        ;;
    esac
    file="start_$1_leopard.launch"
    if [ ! -f "${root_path}/launch/$file" ]; then
        echo "[ERROR]: Could not find '${root_path}/launch/$file'"
    else
        echo "current launch file is: '${root_path}/launch/start_$1_leopard.launch'"
    fi
    NUM_PROCESSES="$(pgrep -c -f "camera_nodelet_manager")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
       if [ $2 ] && [ "$2" = "hup" ]; then
           eval "${CMD} </dev/null >${LOG} 2>&1 &"
       else
           eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
       fi
    fi
}

function stop() {
    pkill -9 -f start_leopard
    pkill -9 -f start_2_leopard
    pkill -9 -f start_3_leopard
    pkill -9 -f start_4_leopard
    pkill -9 -f start_5_leopard
    pkill -9 -f camera_nodelet_manager
}

# run command_name module_name
function run() {
    case $1 in
        start)
            start $2 $3
            ;;
        stop)
            stop $2 $3
            ;; 
        *)
            start $1 $2
            ;;
    esac
}

run "$1" "$2"
