#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_dir/ros2_ws/install/setup.bash"
set -u
pallet_python=${PALLET_PYTHON:-/home/tai/astra_pallets/.venv/bin/python}
if [[ ! -x "$pallet_python" ]]; then
  echo "Missing YOLO Python: $pallet_python" >&2
  exit 1
fi
exec "$pallet_python" "$project_dir/scripts/publish_pallet_target.py" "$@"
