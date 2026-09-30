#!/bin/bash

set -e

BUILD=./WeaklyHard
PLOT=plot_results.py
CONFIGS_DIR=run_configs

TOTAL_START=$SECONDS

for cfg in "$CONFIGS_DIR"/*.json; do
    echo "======================================="
    echo "Running config: $cfg"
    echo "======================================="

    EXP_START=$SECONDS

    $BUILD "$cfg"

    EXP_DIR=$(python3 -c "
import json
with open('$cfg') as f:
    print(json.load(f)['experiment']['results_directory'])
")

    echo "Generating plots..."
    python3 $PLOT "$EXP_DIR"

    EXP_ELAPSED=$((SECONDS - EXP_START))

    echo "Done: $cfg"
    echo "Time for this experiment: ${EXP_ELAPSED}s"
    echo
done

TOTAL_ELAPSED=$((SECONDS - TOTAL_START))

echo "======================================="
echo "All experiments completed."
echo "Total time: ${TOTAL_ELAPSED}s"
echo "======================================="
