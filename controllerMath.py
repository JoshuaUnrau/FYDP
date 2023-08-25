import math

class DronePIDTuner:

    def __init__(self, zeta, tr, I, theta_max, pwm_change, control_freq):
        self.zeta = zeta
        self.tr = tr
        self.I = I
        self.theta_max = theta_max
        self.pwm_change = pwm_change
        self.Ts = 1 / control_freq  # Sampling time

    def compute_wn(self):
        """Compute natural frequency using rise time and damping ratio"""
        return (4.6 / self.tr) / (-self.zeta + math.sqrt(1 - self.zeta**2))

    def compute_pid_gains_desc(self):
        K = self.compute_system_gain()
        wn = self.compute_wn()

        # Standard second-order system relationships to compute PID gains for continuous-time
        Kp_cont = (2 * self.zeta * wn - 1) / K
        Ki_cont = wn**2 / K
        Kd_cont = (1 + 2 * self.zeta * wn) / K

        # Discretize the PID gains using Tustin's method
        Kp = Kp_cont
        Ki = Ki_cont * self.Ts / 2
        Kd = Kd_cont / self.Ts

        return Kp, Ki, Kd

    def compute_pid_gains(self):
        K = self.compute_system_gain()
        wn = self.compute_wn()

        # Standard second-order system relationships to compute PID gains
        Kp = (2 * self.zeta * wn - 1) / K
        Ki = wn ** 2 / K
        Kd = (1 + 2 * self.zeta * wn) / K

        return Kp, Ki, Kd

    def compute_system_gain(self):
        """
        Compute system gain K for a double integrator system.

        Returns:
        - K: System gain
        """

        # Compute angular acceleration
        alpha = 2 * math.radians(self.theta_max) / self.tr**2

        # Compute torque from angular acceleration
        tau = self.I * alpha

        # Determine system gain K
        K = tau / self.pwm_change

        return K

# Example usage:
zeta = 0.1  # Damping ratio
tr = 0.75  # Rise time to 90-degree tilt
I = 0.004987173  # Inertia
theta_max = 90  # 90-degree tilt
pwm_change = 0.6  # example PWM change from the open-loop experiment
control_freq = 100  # Control frequency in Hz

tuner = DronePIDTuner(zeta, tr, I, theta_max, pwm_change, control_freq)
Kp, Ki, Kd = tuner.compute_pid_gains()
print(f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
