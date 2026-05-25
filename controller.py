import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, max_speed, filter_coeff):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed = max_speed
        self.filter_coeff = filter_coeff
        
        self.prev_error = np.zeros(3)
        self.integral = np.zeros(3)
        self.filtered_derivative = np.zeros(3)
        self.current_position = np.array([0.2, 0.2, 0.3])

    def compute(self, smoothed_target, dt):
        error = smoothed_target - self.current_position

        # Tustin Integral
        self.integral += 0.5 * (error + self.prev_error) * dt
        self.integral = np.clip(self.integral, -2, 2)

        # Filtered Derivative
        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (self.filter_coeff * self.filtered_derivative + 
                                    (1 - self.filter_coeff) * raw_derivative)

        # PID Control Signal
        control_signal = (self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative)

        # Speed Clamp
        speed = np.linalg.norm(control_signal)
        if speed > self.max_speed:
            control_signal = (control_signal / speed) * self.max_speed

        # Position Update & Damping
        self.current_position += control_signal * dt
        self.current_position *= 0.998
        
        self.prev_error = error
        return self.current_position