#!/bin/bash

NUM_LOOPS=50

# Run the experiment
for i in $(seq 1 $NUM_LOOPS); do
    python experiment.py
done