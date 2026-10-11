#!/usr/bin/env bash
# Offline only: numerical compilation, configuration and launch XML expansion.
set -eo pipefail
here="${BASH_SOURCE[0]%/*}"
cd "$here/../../.."
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
source /opt/ros/noetic/setup.bash
export PYTHONPYCACHEPREFIX=/tmp/pycache_takeoff_candidate
export PYTHONPATH="$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/src:$PWD/vision_ws/src/uav_high_view/src:$PWD/deployment/board_trials_4x4/common/uav_board_trials/scripts:${PYTHONPATH:-}"
python patrol_uav_ws-patrol_planner/src/patrol_control/test/test_takeoff_xy_hold.py
python patrol_uav_ws-patrol_planner/src/patrol_control/test/test_takeoff_z_lead.py
python patrol_uav_ws-patrol_planner/src/patrol_control/test/test_takeoff_z_lead_config.py
python patrol_uav_ws-patrol_planner/src/uav_mission/test/test_competition_launch_parameters.py
