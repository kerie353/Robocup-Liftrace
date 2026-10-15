#!/usr/bin/env bash
set -euo pipefail
config=/home/lh/.config/liftrace-workbench-sim/sshd_config
mkdir -p /run/sshd
/usr/sbin/sshd -t -f "$config"
if [ -f /home/lh/.config/liftrace-workbench-sim/sshd.pid ] && kill -0 "$(cat /home/lh/.config/liftrace-workbench-sim/sshd.pid)" 2>/dev/null; then exit 0; fi
/usr/sbin/sshd -f "$config"
