#!/bin/bash

set -e

BUILD=./WeaklyHard
PLOT=plot_results.py

if [ $# -ne 1 ]; then
    echo "Usage: ./run_one.sh <config.json>"
    exit 1
fi

cfg=$1

echo "======================================="
echo "Running single experiment: $cfg"
echo "======================================="

START=$SECONDS

# run experiment
$BUILD "$cfg"

# extract output directory
EXP_DIR=$(python3 -c "
import json
with open('$cfg') as f:
    print(json.load(f)['experiment']['results_directory'])
")

echo "Generating plots..."
python3 $PLOT "$EXP_DIR"

ELAPSED=$((SECONDS - START))

echo "======================================="
echo "Done: $cfg"
echo "Results: $EXP_DIR"
echo "Time: ${ELAPSED}s"
echo "======================================="
