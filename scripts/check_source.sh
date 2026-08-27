#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

find src scripts -type f -name '*.py' -not -path '*/__pycache__/*' -print0 |
  xargs -0 -r python3 -m py_compile

find src scripts -type f -name '*.sh' -print0 |
  xargs -0 -r -n1 bash -n

python3 - "$REPO_DIR" <<'PY'
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml

root = Path(sys.argv[1])

for path in root.glob("src/**/package.xml"):
    ET.parse(path)
for path in root.glob("src/**/plugins.xml"):
    # pluginlib permits multiple top-level <library> blocks as a fragment.
    text = re.sub(
        r"^\s*<\?xml[^?]*\?>", "", path.read_text(encoding="utf-8")
    )
    ET.fromstring("<plugin_libraries>" + text +
                  "</plugin_libraries>")
for path in root.glob("src/**/*.yaml"):
    with path.open(encoding="utf-8") as stream:
        yaml.safe_load(stream)

print("XML and YAML parsing passed")
PY

grep -Eq '^[[:space:]]*enable_motion:[[:space:]]*false' \
  src/g1_nav_control/config/velocity_bridge.yaml

if grep -R -n -E '/home/[^/]+/' --exclude='*.bak' \
  src/g1_nav_sim/env src/g1_nav_slam/launch src/g1_nav_slam/save_map.sh; then
  echo "Machine-specific absolute home path found" >&2
  exit 1
fi

echo "Source checks passed"
