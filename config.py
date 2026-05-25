import numpy as np

# =========================================================
# CONFIGURATION
# =========================================================

MODEL_PATH = "hand_landmarker.task"

# EMA smoothing
ALPHA = 0.18

# Tustin PID gains
KP = 4.0
KI = 0.8
KD = 0.6

# Derivative low-pass filter
DERIVATIVE_FILTER = 0.85

# Maximum arm speed
MAX_SPEED = 8.0

# Gripper threshold
PINCH_THRESHOLD = 0.05

# Arm lengths: [Base Height (L0), Shoulder-to-Elbow (L1), Elbow-to-Wrist (L2)]
SEGMENT_LENGTHS = [0.1, 0.45, 0.35]

# Base position
BASE_POSITION = np.array([0.0, 0.0, 0.0])

# Dashboard history buffer size
HISTORY = 50