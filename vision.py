import cv2
import time
import math
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python import vision

import config
from kinematics import calculate_ik_3dof
from controller import PIDController

class VisionTracker:
    def __init__(self, state):
        self.state = state
        self.latest_result = None
        self.pid_controller = PIDController(config.KP, config.KI, config.KD, 
                                            config.MAX_SPEED, config.DERIVATIVE_FILTER)
        
        # Setup MediaPipe
        base_options = mp.tasks.BaseOptions(model_asset_path=config.MODEL_PATH)
        self.options = vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_hands=1,
            result_callback=self._result_callback
        )

    def _result_callback(self, result, output_image, timestamp_ms):
        self.latest_result = result

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        smoothed_target = np.zeros(3)
        last_time = time.time()
        start_time = time.time()

        with vision.HandLandmarker.create_from_options(self.options) as landmarker:
            while self.state.running:
                ret, frame = cap.read()
                if not ret: continue

                frame = cv2.flip(frame, 1)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
                
                landmarker.detect_async(mp_image, int(time.time() * 1000))
                h, w, _ = frame.shape
                
                gripper_state_text = "OPEN"
                joints = self.state.joints.copy()
                angles = self.state.angles.copy()
                current_position = self.pid_controller.current_position

                if self.latest_result and self.latest_result.hand_landmarks:
                    hand_landmarks = self.latest_result.hand_landmarks[0]
                    wrist = hand_landmarks[0]
                    index_mcp = hand_landmarks[5]
                    thumb_tip = hand_landmarks[4]
                    index_tip = hand_landmarks[8]

                    # Depth & Space Mapping
                    hand_size_2d = math.sqrt((wrist.x - index_mcp.x) ** 2 + (wrist.y - index_mcp.y) ** 2)
                    target_z = 0.12 / (hand_size_2d + 1e-6)
                    
                    world_x = (wrist.x - 0.5) * 1.2
                    world_y = (0.5 - wrist.y) * 1.2
                    world_z = np.clip(target_z, 0.1, 1.0)
                    raw_target = np.array([world_x, world_y, world_z])

                    # Filters & Timing
                    smoothed_target = (config.ALPHA * raw_target + (1 - config.ALPHA) * smoothed_target)
                    
                    current_time = time.time()
                    dt = max(current_time - last_time, 0.001)
                    last_time = current_time

                    # Update Controller & Kinematics
                    current_position = self.pid_controller.compute(smoothed_target, dt)
                    joints, angles = calculate_ik_3dof(current_position, config.SEGMENT_LENGTHS)

                    # Gripper Logic
                    pinch_dist = math.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
                    gripper_state_text = "CLOSED" if pinch_dist < config.PINCH_THRESHOLD else "OPEN"

                    # OpenCV Drawing
                    cv2.circle(frame, (int(wrist.x * w), int(wrist.y * h)), 10, (255, 0, 0), -1)
                    color = (0, 0, 255) if gripper_state_text == "CLOSED" else (0, 255, 0)
                    cv2.circle(frame, (int(thumb_tip.x * w), int(thumb_tip.y * h)), 8, color, -1)
                    cv2.circle(frame, (int(index_tip.x * w), int(index_tip.y * h)), 8, color, -1)
                    cv2.line(frame, (int(thumb_tip.x * w), int(thumb_tip.y * h)), 
                             (int(index_tip.x * w), int(index_tip.y * h)), color, 3)

                # State Synchronization
                run_time = time.time() - start_time
                with self.state.lock:
                    self.state.smoothed_target = smoothed_target.copy()
                    self.state.current_position = current_position.copy()
                    self.state.joints = [np.copy(j) for j in joints]
                    self.state.angles = list(angles)
                    self.state.gripper_state = gripper_state_text
                    
                    self.state.timestamps.append(run_time)
                    for i in range(3):
                        self.state.targets[i].append(smoothed_target[i])
                        self.state.positions[i].append(current_position[i])

                # HUD Overlay
                cv2.rectangle(frame, (0, 0), (640, 100), (0, 0, 0), -1)
                cv2.putText(frame, f"TARGET : X:{smoothed_target[0]:+.2f} Y:{smoothed_target[1]:+.2f} Z:{smoothed_target[2]:.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 180), 1)
                cv2.putText(frame, f"PID ARM: X:{current_position[0]:+.2f} Y:{current_position[1]:+.2f} Z:{current_position[2]:.2f}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(frame, f"SERVOS : B:{angles[0]:+.1f} S:{angles[1]:+.1f} E:{angles[2]:+.1f} (Deg)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 255), 1)
                hud_color = (0, 0, 255) if gripper_state_text == "CLOSED" else (0, 255, 0)
                cv2.putText(frame, f"GRIPPER: {gripper_state_text}", (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hud_color, 1)

                cv2.imshow("Hand Tracking Control", frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    self.state.running = False
                    break

        cap.release()
        cv2.destroyAllWindows()