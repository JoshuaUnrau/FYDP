from evdev import InputDevice, categorize, ecodes
import threading
import asyncio

class Joystick:

    def __init__(self, device_path='/dev/input/event0'):
        self.device = InputDevice(device_path)

        # Initial drone settings
        self.pitch, self.roll, self.yaw = 0, 0, 0
        self.fl_motor, self.fr_motor = 0, 0
        self.rl_motor,  self.rr_motor = 0, 0
        self.throttle = 0
        self.P, self.I, self.D = 0, 0, 0
        self.shutdown = False
        self.direct_control = False

        # Zeroing offsets
        self.pitch_offset, self.roll_offset, self.yaw_offset = 0, 0, 0

        # Previous joystick values for low-pass filtering
        self.prev_pitch, self.prev_roll, self.prev_yaw = 0, 0, 0

        # Last logged joystick values
        self.last_logged_pitch, self.last_logged_roll, self.last_logged_yaw = 0, 0, 0
        self.threshold = 100  # Set the threshold

        self.alpha = 0.1  # Smoothing factor for low-pass filter
        self.loop_stop_event = threading.Event()  # An event to signal the joystick loop to stop.op to stop.
        self.run()
        print("xbox controller connected")

    def run(self):
        self.loop_thread = threading.Thread(target=self.joystick_loop)
        self.loop_thread.start()

    def joystick_loop(self,):
        print("Reading joystick")
        while not self.loop_stop_event.is_set():
            for event in self.device.read_loop():
                if self.loop_stop_event.is_set():  # Check if we need to stop the loop.
                    print("Joystick loop terminated.")
                    return
                if event.type == ecodes.EV_KEY:
                    self._handle_key_event(event)
                elif event.type == ecodes.EV_ABS:
                    self._handle_abs_event(event)

    async def stop(self):
        self.loop_stop_event.set()  # Signal the joystick loop to sto

    def _handle_key_event(self, event):
        if event.code == ecodes.BTN_A and event.value == 1:
            self.direct_control = not self.direct_control
            print(f"Direct control: {self.throttle}")

        elif event.code == ecodes.BTN_B and event.value == 1:
            self.shutdown = False
            print(f"Powering on")

        elif event.code == ecodes.BTN_X and event.value == 1:
            self.throttle = 0
            self.shutdown = True
            self.pitch, self.roll, self.yaw = 0, 0, 0
            print("EMERGENCY STOP! Drone shut down.")

        elif event.code == ecodes.BTN_THUMBL and event.value == 1:
            self.pitch_offset = self.pitch
            self.roll_offset = self.roll
            print("Joystick values zeroed.")

        elif event.code == ecodes.BTN_THUMBR and event.value == 1:
            self.yaw_offset = self.yaw
            print("Joystick yaw value zeroed.")

        # Handle left trigger
        elif event.code == ecodes.ABS_Z:
            self.fl_motor = event.value

        # Handle right trigger
        elif event.code == ecodes.ABS_RZ:
            self.fr_motor = event.value

    def _handle_abs_event(self, event):
        if event.code == ecodes.ABS_X:
            self.roll = self.alpha * (event.value - self.roll_offset) + (1 - self.alpha) * self.prev_roll
            self.prev_roll = self.roll
            if abs(self.roll - self.last_logged_roll) > self.threshold:
                self.last_logged_roll = self.roll
                print(f"Roll: {self.roll}")

        elif event.code == ecodes.ABS_Y:
            self.rl_motor = event.value
            self.pitch = self.alpha * (event.value - self.pitch_offset) + (1 - self.alpha) * self.prev_pitch
            self.prev_pitch = self.pitch
            if abs(self.pitch - self.last_logged_pitch) > self.threshold:
                self.last_logged_pitch = self.pitch
                print(f"Pitch: {self.pitch}")

        elif event.code == ecodes.ABS_RX:
            self.yaw = self.alpha * (event.value - self.yaw_offset) + (1 - self.alpha) * self.prev_yaw
            self.prev_yaw = self.yaw
            if abs(self.yaw - self.last_logged_yaw) > self.threshold:
                self.last_logged_yaw = self.yaw
                print(f"Yaw: {self.yaw}")

        elif event.code == ecodes.ABS_RY:
            self.rr_motor = event.value
            self.yaw = self.alpha * (event.value - self.yaw_offset) + (1 - self.alpha) * self.prev_yaw
            self.prev_yaw = self.yaw
            if abs(self.yaw - self.last_logged_yaw) > self.threshold:
                self.last_logged_yaw = self.yaw
                print(f"Yaw: {self.yaw}")

        elif event.code == ecodes.ABS_HAT0X:
            if event.value == 1:
                self.P += 0.1
            elif event.value == -1:
                self.P -= 0.1
            print(f"P: {self.P}, I: {self.I}, D: {self.D}")

        elif event.code == ecodes.ABS_HAT0Y:
            if event.value == 1:
                self.D += 0.1
            elif event.value == -1:
                self.D -= 0.1
            print(f"P: {self.P}, I: {self.I}, D: {self.D}")