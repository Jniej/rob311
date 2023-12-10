import numpy as np
import sys
from constants import *

from pyPS4Controller.controller import Controller
import time

class BBController(Controller):
    """
    This controller class can be implemented in many different ways and this is one of them.
    """
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.MAX_TZ = 0.5 # Nm
        self.MAX_VELOCITY = 0.85 # rad/sec

        self.DELTA_KP = 0.1
        self.DELTA_KD = 0.01

        self.MAX_ROTATION_TIME = 0.75 # Sec
        self.Tz = 0.0
        self.Tx = 0.0
        self.Ty = 0.0

        self.dphi_y_sp = 0.0
        self.dphi_x_sp = 0.0

        self.theta_kp = 9.0
        self.theta_ki = 0.0
        self.theta_kd = 0.1

        self.COOLDOWN = 0.5
        self.MAX_ROTATION_ITER = int(self.MAX_ROTATION_TIME/DT)

        self.refrence_roll = 0.0
        self.refrence_pitch = 0.0

    def on_L3_up(self, value):
        #TODO: increase velocity
        self.Tx = -0.25 * (value/JOYSTICK_SCALE)

        #print("L3_up: ", value)

    def on_L3_down(self, value):
        #TODO: decrease velocity
        self.Tx = -0.25 * (value/JOYSTICK_SCALE)

        #print("L3_down: ", value)

    def on_L3_right(self, value):
        self.Ty = -0.25 * (value/JOYSTICK_SCALE)

    def on_L3_left(self, value):
        self.Ty = -0.25 * (value/JOYSTICK_SCALE)

    def on_L3_y_at_rest(self):
        #TODO: set velocity to 0
        self.Tx = 0.0
        self.Ty = 0.0

    def on_R3_up(self, value):
        #TODO: increase velocity
        self.dphi_y_sp = -3 * (value/JOYSTICK_SCALE) * self.MAX_VELOCITY 

        #print("L3_up: ", value)

    def on_R3_down(self, value):
        #TODO: decrease velocity
        self.dphi_y_sp = -3 * (value/JOYSTICK_SCALE) * self.MAX_VELOCITY

        #print("L3_down: ", value)

    def on_R3_right(self, value):
        self.dphi_x_sp = -3 * (value/JOYSTICK_SCALE) * self.MAX_VELOCITY

    def on_R3_left(self, value):
        self.dphi_x_sp = -3 * (value/JOYSTICK_SCALE) * self.MAX_VELOCITY

    def on_R3_y_at_rest(self):
        #TODO: set velocity to 0
        self.dphi_y_sp = 0.0
        self.dphi_x_sp = 0.0
    
    def on_R2_press(self, value):
        self.dphi_y_sp = 2.0 * self.MAX_VELOCITY * (1.0 + value/JOYSTICK_SCALE)/2.0

    def on_R2_release(self):
        # TODO: Release mechanism
        #while abs(self.dphi_x_sp) > 0.0:
        #    self.dphi_x_sp -= 0.01
        #    time.sleep(DT)

        #while abs(self.dphi_y_sp) > 0.0:
        #    self.dphi_y_sp -= 0.01
        #    time.sleep(DT)

        self.dphi_y_sp = 0.0
        self.dphi_x_sp = 0.0

    def on_L2_press(self, value):
        self.dphi_y_sp = -2.0 * self.MAX_VELOCITY * (1.0 + value/JOYSTICK_SCALE)/2.0

    def on_L2_release(self):
        # TODO: Release mechanism
        self.dphi_y_sp = 0.0

    def on_R1_press(self):
        for i in range(0, self.MAX_ROTATION_ITER):
            self.Tz = 1.7 * self.MAX_TZ * np.sin(i) 
            time.sleep(DT)

        time.sleep(self.COOLDOWN)
    
    def on_R1_release(self):
        self.Tz = 0.0

    def on_L1_press(self):
        for i in range(0, self.MAX_ROTATION_ITER):
            self.Tz = -0.7 * self.MAX_TZ * np.sin(i) 
            time.sleep(DT)

        time.sleep(self.COOLDOWN)
    
    def on_L1_release(self):
        self.Tz = 0.0

    def on_triangle_press(self):
        self.refrence_roll = 0.1
        self.refrence_pitch = 0.1
        pass

    def on_triangle_release(self):
        pass

    def on_x_press(self):
        self.dphi_x_sp = 1.5
        self.dphi_y_sp = 1.5


    def on_x_release(self):
        pass

    def on_circle_press(self):
        self.dphi_x_sp = 0.0
        self.dphi_y_sp = 0.0

    def on_circle_release(self):
        pass

    def on_square_press(self):
        self.refrence_roll = 0.0
        self.refrence_pitch = 0.0

    def on_square_release(self):
        pass

    def on_options_press(self):
        print("Exiting controller thread.")
        sys.exit()
