#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG="${SCRIPT_DIR}/../packages/pf_localization_real/rviz/pf_loc.rviz"

if [ ! -f "${CONFIG}" ]; then
    echo "rviz.sh: config not found at ${CONFIG}" >&2
    echo "         did you forget '--mount \"\$(pwd)\"' on start_gui_tools?" >&2
    exit 1
fi

exec rviz -d "${CONFIG}"
