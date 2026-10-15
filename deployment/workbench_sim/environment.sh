#!/usr/bin/env bash
# Isolated laptop simulation environment; never sourced by a hardware profile.
sim_root="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
export PX4_ROOT=/home/lh/PX4-Autopilot-TouDi
export PX4_SYS_AUTOSTART=10015
export ASTRA_LIB=/home/lh/patrol_sim_ws/devel/lib
export ASTRA_SIM_LIB="$ASTRA_LIB"
export ASTRA_MODEL_ROOT="$sim_root/simulation_assets/models"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
# SSH does not inherit WSLg's display. Gazebo cameras need rendering even
# when the GUI is disabled.
export DISPLAY="${DISPLAY:-:0}"
export BOARD_PYTHON=/usr/bin/python3
export UAV_RESEARCH_PYTHON=/usr/bin/python3
# Fill these once the trained .pt model and inference environment are supplied.
export VISION_PYTHON="${VISION_PYTHON:-/home/lh/.venvs/liftrace-workbench-sim/bin/python}"
export UAV_VISION_MODEL_PATH="${UAV_VISION_MODEL_PATH:-$sim_root/runtime_models/workbench_sim.pt}"
export SIM_NO_RECORD=1
export SIM_STORAGE_GUARD_PATH=/mnt/f
export GAZEBO_MODEL_PATH="$sim_root/vision_ws/src/uav_vision_eval/models:$ASTRA_MODEL_ROOT${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
source /opt/ros/noetic/setup.bash
if [ -f "$sim_root/vision_ws/devel/setup.bash" ]; then source "$sim_root/vision_ws/devel/setup.bash"; fi
if [ -f "$sim_root/patrol_uav_ws-patrol_planner/devel/setup.bash" ]; then source "$sim_root/patrol_uav_ws-patrol_planner/devel/setup.bash" --extend; fi
