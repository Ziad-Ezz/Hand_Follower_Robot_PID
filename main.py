import time
import threading
from robot_state import SharedState
from vision import VisionTracker
from dashboard import run_dashboard

def main():
    # 1. Initialize Thread-Safe State
    state = SharedState()
    
    # 2. Initialize Vision System
    tracker = VisionTracker(state)
    
    # 3. Start OpenCV & Logic in a Daemon Thread
    cv_thread = threading.Thread(target=tracker.run, daemon=True)
    cv_thread.start()
    
    # Let the camera warm up
    time.sleep(1.5)
    
    # 4. Start the Matplotlib Dashboard on the Main Thread
    run_dashboard(state)
    
    # 5. Clean up when Dashboard is closed
    state.running = False
    cv_thread.join(timeout=3)

if __name__ == "__main__":
    main()