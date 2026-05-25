# 🤖 Vision-Guided 3-DOF Robotic Arm Control System

A real-time, vision-guided robotic arm control platform developed for advanced Numerical Analysis and Mechatronics applications.  
This project integrates **computer vision**, **analytical inverse kinematics**, and a **custom Tustin PID controller** to translate human hand motion into smooth and responsive robotic arm movement in 3D space.

Using a standard webcam and Google's MediaPipe HandLandmarker, the system tracks a user’s hand in real time and maps spatial coordinates directly to a 3-DOF robotic manipulator while maintaining low-latency motion, trajectory stability, and live telemetry visualization.

---

# 📌 Overview

The system was engineered to address one of the most challenging aspects of low-cost robotic teleoperation systems:

- Stable real-time motion tracking
- Smooth servo actuation without jitter
- Efficient inverse kinematics computation
- Reliable depth estimation using monocular vision
- Thread-safe visualization and telemetry rendering

Unlike conventional implementations that rely on iterative IK solvers and noisy raw tracking data, this project combines:

- **Analytical trigonometric inverse kinematics**
- **Tustin-based PID control**
- **Filtered derivative response**
- **Dynamic Z-axis estimation**
- **Multi-threaded system architecture**

to produce a highly responsive and computationally efficient robotic control framework.

---

# ✨ Key Features

## 🎯 Real-Time Vision-Based Hand Tracking
- Uses **Google MediaPipe HandLandmarker** for high-speed 3D hand landmark detection
- Converts hand motion into robotic target coordinates in real time
- Supports continuous motion tracking with low latency

---

## ⚙️ Custom Tustin PID Controller

A fully custom PID implementation designed specifically for robotic servo stabilization.

### Features
- Proportional–Integral–Derivative control
- **Tustin (bilinear) integration**
- Low-pass filtered derivative term
- Servo jitter suppression
- Smooth coordinate convergence
- Stable transient response

### Advantages
- Eliminates oscillation caused by noisy webcam data
- Produces significantly smoother robotic motion
- Improves trajectory stability under rapid hand movement

---

## 📐 Analytical 3-DOF Inverse Kinematics

The robotic arm uses a fully analytical inverse kinematics solution for:

- Base Yaw
- Shoulder Pitch
- Elbow Pitch

### Technical Highlights
- Solves joint angles using trigonometric closed-form equations
- Avoids computationally expensive iterative solvers
- Optimized for real-time execution
- Enables fast control loop updates

This approach dramatically reduces computational overhead while maintaining precise arm positioning.

---

## 📏 Dynamic Z-Axis Depth Tracking

One of the core engineering challenges solved in this project.

Since a monocular webcam cannot directly measure depth, the system dynamically estimates Z-axis distance using:

- A 2D hand-size proxy
- Landmark scaling relationships
- Adaptive spatial mapping

### Result
- Accurate forward/backward hand tracking
- Stable spatial positioning
- Natural 3D interaction using a single camera

---

## 📊 Live Telemetry Dashboard

A dedicated multi-threaded Matplotlib telemetry interface provides:

### Real-Time Visualization
- 3D robotic arm rendering
- Live target waypoint plotting
- PID trajectory tracking
- Servo state visualization

### Engineering Importance
The visualization subsystem was strictly preserved while decoupling the computational logic into modular threads, ensuring:

- Non-blocking UI rendering
- Stable frame rates
- Cleaner system scalability
- Better maintainability

---

## 🤏 Pinch-to-Grip Gesture Control

Implements gesture-based gripper actuation.

### Gesture Detection
- Detects thumb-index pinch distance
- Automatically toggles gripper states

### Benefits
- Natural user interaction
- Intuitive robotic manipulation
- Contactless control interface

---

## 🧩 Modular Multi-Threaded Architecture

The entire project follows strict **Separation of Concerns (SoC)** principles.

### Dedicated Modules
- Configuration management
- Shared state management
- Inverse kinematics engine
- PID controller
- Vision processing pipeline
- Dashboard rendering
- Hardware communication

### Benefits
- Scalable codebase
- Easier debugging
- Improved maintainability
- Safer thread synchronization
- Clear subsystem isolation

---

# 🏗️ Project Structure

```text
robotic-arm-control/
│
├── config/
│   └── settings.py
│
├── core/
│   ├── state_manager.py
│   ├── controller.py
│   ├── inverse_kinematics.py
│   └── filters.py
│
├── vision/
│   ├── hand_tracker.py
│   ├── depth_estimation.py
│   └── gesture_detection.py
│
├── dashboard/
│   ├── telemetry_ui.py
│   └── visualizer_3d.py
│
├── hardware/
│   ├── servo_interface.py
│   └── gripper_control.py
│
├── main.py
├── requirements.txt
└── README.md
```

---

# 🛠️ Technologies Used

- Python 3.x
- OpenCV
- MediaPipe
- NumPy
- Matplotlib
- Multi-threading
- Analytical Robotics Kinematics
- PID Control Systems

---

# 📋 Prerequisites

Before running the project, ensure the following are installed:

- Python 3.9+
- Webcam device
- pip package manager

Recommended:
- Virtual environment (`venv`)
- GPU acceleration (optional)

---

# 🚀 Installation

## 1️⃣ Clone the Repository

```bash
git clone https://github.com/your-username/robotic-arm-control.git
cd robotic-arm-control
```

---

## 2️⃣ Create a Virtual Environment

### Windows
```bash
python -m venv venv
venv\Scripts\activate
```

### Linux / macOS
```bash
python3 -m venv venv
source venv/bin/activate
```

---

## 3️⃣ Install Dependencies

```bash
pip install -r requirements.txt
```

---

# ▶️ Usage

Run the main application:

```bash
python main.py
```

---

# 🎮 Runtime Interface

## 🖥️ OpenCV Vision HUD

The OpenCV window displays:

- Live webcam feed
- Hand landmark tracking
- Real-time target coordinates
- Gesture recognition state
- Tracking overlays

### Interaction
- Move your hand to control robotic arm positioning
- Move closer/farther from the camera for Z-axis control
- Perform a pinch gesture to actuate the gripper

---

## 📊 Telemetry Dashboard

The Matplotlib dashboard provides live engineering telemetry.

### Displays
- 3D robotic arm model
- Current end-effector position
- Target waypoint visualization
- PID tracking response
- Motion smoothing behavior

This dashboard was intentionally architected as a decoupled subsystem to maintain UI responsiveness independently from the control loop.

---

# 🧠 Control Pipeline

```text
Webcam Input
      ↓
MediaPipe Hand Tracking
      ↓
Coordinate Extraction
      ↓
Dynamic Depth Estimation
      ↓
Target Position Mapping
      ↓
Tustin PID Controller
      ↓
Analytical Inverse Kinematics
      ↓
Servo Command Generation
      ↓
Robotic Arm Motion
```

---

# 📈 Engineering Objectives

This project was developed to demonstrate:

- Numerical Analysis implementation in robotics
- Real-time control systems engineering
- Computer vision integration
- Practical inverse kinematics
- Signal filtering and stabilization
- Multi-threaded software architecture
- Human-machine interaction systems

---

# 🔬 Future Improvements

Potential future enhancements include:

- 6-DOF arm expansion
- Kalman filtering for advanced tracking
- ROS integration
- Reinforcement learning assisted motion
- Stereo vision depth estimation
- Trajectory prediction
- Hardware acceleration using CUDA

---

# 👨‍💻 Authors

### 👤 Ziad Ahmed Ezz
Mechatronics Engineering Student  

### 👤 Mohammed Nasser
Mechatronics Engineering Student

### 👤 Sherif Ahmed
Mechatronics Engineering Student

---

# 📜 License

This project is intended for educational, research, and academic purposes.
