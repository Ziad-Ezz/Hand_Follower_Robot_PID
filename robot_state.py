import threading
import numpy as np
from collections import deque
from config import BASE_POSITION, SEGMENT_LENGTHS, HISTORY

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.smoothed_target = np.zeros(3)
        self.current_position = np.array([0.2, 0.2, 0.3])
        self.angles = [0.0, 0.0, 0.0]
        
        # Initializing the 4 joints (Base, Shoulder, Elbow, Wrist)
        self.joints = [
            BASE_POSITION,
            BASE_POSITION + np.array([0, 0, SEGMENT_LENGTHS[0]]),
            BASE_POSITION + np.array([0, SEGMENT_LENGTHS[1], SEGMENT_LENGTHS[0]]),
            BASE_POSITION + np.array([0, SEGMENT_LENGTHS[1] + SEGMENT_LENGTHS[2], SEGMENT_LENGTHS[0]])
        ]
        
        self.gripper_state = "OPEN"
        self.running = True
        
        # Dashboard logging
        self.timestamps = deque(maxlen=HISTORY)
        self.targets = [deque(maxlen=HISTORY) for _ in range(3)]
        self.positions = [deque(maxlen=HISTORY) for _ in range(3)]