#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
root="$(cd "$script_dir/../.." && pwd)"
# The test-registration workaround is permitted only in this isolated clone.
[[ "$root" == /home/lh/liftrace-workbench-sim-20261014 ]] || { echo 'Use the isolated WSL simulation clone'; exit 2; }
/usr/bin/python3 - "$root" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])/'patrol_uav_ws-patrol_planner/src/uav_mission/CMakeLists.txt'
s = p.read_text()
line = '  catkin_add_nosetests(test/test_release_invocation_recovery.py)\n'
count = s.count(line)
if count == 2:
    i = s.rfind(line)
    p.write_text(s[:i]+s[i+len(line):])
elif count != 1:
    raise RuntimeError('Review changed test registration before building')
PY
mkdir -p "$root/logs"
BUILD_JOBS="${BUILD_JOBS:-2}" bash "$root/top_level_scripts/build_competition.sh" >"$root/logs/workbench_build.log" 2>&1
