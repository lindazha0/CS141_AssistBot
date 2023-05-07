#!/bin/bash

NUM_LOOPS=40

# Run the experiment
for i in $(seq 1 $NUM_LOOPS); do
    python experiment.py
done