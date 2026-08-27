#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

find src/b2_driver src/b2_navigation src/nav2_custom_plugins scripts experiments \
  -type f -name '*.py' -not -path '*/__pycache__/*' -print0 |
  xargs -0 -r python3 -m py_compile

find scripts experiments -type f -name '*.sh' -print0 |
  xargs -0 -r -n1 bash -n

python3 - "$REPO_DIR" <<'PY'
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml

root = Path(sys.argv[1])
packages = (
    root / "src/b2_driver",
    root / "src/b2_navigation",
    root / "src/nav2_custom_plugins",
)

for package in packages:
    ET.parse(package / "package.xml")
for path in (root / "src/nav2_custom_plugins").glob("*.xml"):
    # pluginlib permits multiple top-level <library> blocks as a fragment.
    text = re.sub(
        r"^\s*<\?xml[^?]*\?>", "", path.read_text(encoding="utf-8")
    )
    ET.fromstring("<plugin_libraries>" + text +
                  "</plugin_libraries>")
for package in packages:
    for path in package.rglob("*.yaml"):
        with path.open(encoding="utf-8") as stream:
            yaml.safe_load(stream)

print("XML and YAML parsing passed")
PY

grep -q 'START_WALK=false' scripts/start_b2_navigation.sh
grep -q 'declare_parameter<bool>("enable_motion", false)' \
  src/b2_driver/src/b2_walk.cpp

if grep -R -n -E '/home/[^/]+/' --exclude=check_source.sh --exclude='*.bak' scripts; then
  echo "Machine-specific absolute home path found in public scripts" >&2
  exit 1
fi

echo "Source checks passed"
