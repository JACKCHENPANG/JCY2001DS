#!/usr/bin/env bash
# Isolated PlatformIO Core on Mac internal APFS; the normal ~/.platformio is
# kept untouched for the V0.5.2 rollback environment.
set -euo pipefail
export PLATFORMIO_CORE_DIR="/Users/jcy/Library/Caches/JCY1000-5-platformio-idf55"
exec /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 -m platformio "$@"
