#!/bin/bash
# Script to run diag_gait.py with given parameters

python3 diag_gait.py \
  --port /dev/ttyAMA0 \
  --speed 2800 \
  --acc 70 \
  --height 35 \
  --lift 60 \
  --step 10 \
  --stride-time 1.0
