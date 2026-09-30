#!/usr/bin/env bash

set -e

for dir in experiments/*; do
    if [ -d "$dir" ]; then
        echo "Plotting $dir"
        python3 plot_results.py "$dir"
    fi
done
