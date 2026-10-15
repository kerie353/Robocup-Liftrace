#!/usr/bin/env bash
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
set +u
source "$script_dir/environment.sh"
set -u
exec /usr/bin/python3 "$script_dir/session.py" "$@"
