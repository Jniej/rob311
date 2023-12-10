"""
ROB 311 - Ball-bot steering boilerplate code

Authors: Senthur Ayyappan, Japmanjeet Singh Gill, and Elliott Rouse 
Neurobionics Lab / Locomotor Control Lab
"""

import sys
from threading import Thread
import time
import numpy as np

from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol

from collections import deque
from DataLogger import dataLogger

from loop import SoftRealtimeLoop
from ps4 import BBController

from constants import *

from transformations import transform_w2b, compute_motor_torques

import FIR as fir

# Initializing lowpass filters for the ball-velocity estimates.
# Please note that these filters can be modified to filter the IMU data, 
# ball velocity, or ball position. 
lowpass_filter_dphi_x = fir.FIR()
lowpass_filter_dphi_x.lowpass(N, Fn)

lowpass_filter_dphi_y = fir.FIR()
lowpass_filter_dphi_y.lowpass(N, Fn)

def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

def wma_filter(wma_window):
    return np.sum(WMA_NORM * wma_window)

if __name__ == "__main__":

    trial_num = int(input('Trial Number? '))
    filename = 'ROB311_Test_%i' % trial_num
    dl = dataLogger(filename + '.txt')

    t_start = 0.0

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Initializing a thread for reading data from the Pico
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local data structures for storing the data obtained from the Pico
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    # Local variables for the controller
    psi = np.zeros((3, 1))
    psi_offset = np.zeros((3, 1))

    dpsi = np.zeros((3, 1))

    phi = np.zeros((3, 1))
    dphi = np.zeros((3, 1))
    prev_phi = phi

    theta_x = 0.0
    theta_y = 0.0

    dphi_x = 0.0
    dphi_y = 0.0

    # deque is a data structure that automatically pops the previous data based on its max length.
    theta_x_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values
    theta_y_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values

    for _ in range(WMA_WINDOW_SIZE):
        theta_x_window.append(0.0)
        theta_y_window.append(0.0)

    # Net Tx, Ty, and Tz 
    Tx = 0.0
    Ty = 0.0
    Tz = 0.0

    # Steering controller torques: Tx_steer, Ty_steer, and Tz_steer
    Tx_steer = 0.0
    Ty_steer = 0.0
    Tz_steer = 0.0

    # Stability controller torques: Tx_sta, Ty_sta, and Tz_sta
    Tx_bal = 0.0
    Ty_bal = 0.0
    Tz_bal = 0.0

    # T1, T2, and T3
    T1 = 0.0
    T2 = 0.0
    T3 = 0.0

    commands['start'] = 1.0
    zeroed = False

    # Time for communication between the RPi and Pico to be established
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)

    # Set points for the stability controller (along x|roll and y|pitch).
    theta_roll_sp = 0.0
    theta_pitch_sp = 0.0

    # Error in theta (for stability controller)
    error_x = 0.0
    error_y = 0.0
    error_int_x = 0.0
    error_int_y = 0.0
    i_term_x = 0.0
    i_term_y = 0.0

    # Set points for the steering controller (along x|roll and y|pitch).
    dphi_roll_sp = 0.0
    dphi_pitch_sp = 0.0

    
    
    print('Starting the controller!')
    i = 0

    # This thread runs in parallel to the main controller loop and listens for any PS4 input
    # from the user. The PS4 controller is used to update setpoints the steering controller and to tune 
    # the controller gains on the fly. Please refer to the ps4.py file for more details.
    bb_controller = BBController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    bb_controller_thread = Thread(target=bb_controller.listen, args=(10,))
    bb_controller_thread.start()

    # Entering our main control loop, which is set to run at 200 Hz.
    # Feel free to experiment with the frequency of the controller loop by 
    # changing the value of "FREQ" variable in constants.py.
    for t in SoftRealtimeLoop(dt=DT, report=True):

        # Reading data from the Pico, if it isn't available, then skip this iteration.
        # This is true when the Pico is connected and is collecting the IMU offsets.
        try:
            states = ser_dev.get_cur_topic_data(121)[0]

        except KeyError as e:
            # Calibrates for 10 seconds
            print("<< CALIBRATING :: {:.2f} >>".format(t))
            continue

        # Extracting the Motor Encoder values from the Pico's data
        psi[0] = states['psi_1']
        psi[1] = states['psi_2']
        psi[2] = states['psi_3']

        dpsi[0] = states['dpsi_1']
        dpsi[1] = states['dpsi_2']
        dpsi[2] = states['dpsi_3']

        # A sliding window of "WMA_WINDOW_SIZE" values for the WMA filter
        # on the IMU values. Please edit the constants.py file to change the
        # WMA_WINDOW_SIZE and the WMA_WEIGHTS.
        theta_x_window.append(states['theta_roll'])
        theta_y_window.append(states['theta_pitch'])

        # Applying a WMA filter on the IMU values
        theta_x = wma_filter(theta_x_window)
        theta_y = wma_filter(theta_y_window)

        # A ten second wait to place the bot on top of the ball--this is to 
        # reset the encoder values so that at i=0, the ball-bot's position is (0, 0)
        if t > 11.0 and t < 21.0:
            print("<< PLACE THE BOT ON TOP OF THE BALL :: {:.2f} >>".format(t))
        elif t > 21.0:
            if not zeroed:
                psi_offset = psi
                zeroed = True

        psi = psi - psi_offset

        # Transforming wheel attributes (position and velocity) to ball attributes.
        phi[0], phi[1], phi[2] = transform_w2b(psi[0], psi[1], psi[2])
        dphi[0], dphi[1], dphi[2] = transform_w2b(dpsi[0], dpsi[1], dpsi[2])

        # Lowpass filtering the ball-velocity estimates
        dphi_x = lowpass_filter_dphi_x.filter(dphi[0][0])
        dphi_y = lowpass_filter_dphi_y.filter(dphi[1][0])

        # Few conditional statements to start the time counter only
        # after the ball-bot is placed on top of the ball.
        if zeroed:
            if i == 0:
                t_start = time.time()

            i = i + 1
            t_now = time.time() - t_start

        # PID - Balance Controller (along x|roll and y|pitch).
        # Your PID implementation goes here 
        error_x = theta_roll_sp - theta_x
        error_y = theta_pitch_sp - theta_y

        error_int_x += error_x
        error_int_y += error_y

        i_term_x = KI_THETA_X * error_int_x #* DT
        i_term_y = KI_THETA_Y * error_int_y #* DT
        if np.abs(i_term_x) > 0.5 * MAX_STA_DUTY:
            i_term_x = np.sign(i_term_x) * 0.5 * MAX_STA_DUTY
        if np.abs(i_term_y) > 0.5 * MAX_STA_DUTY:
            i_term_y = np.sign(i_term_y) * 0.5 * MAX_STA_DUTY

        Tx = KP_THETA_X * error_x + i_term_x
        Ty = KP_THETA_Y * error_y + i_term_y

        # Changing Tz with R2 and L2 on controller (turning the bot)
        Tz = bb_controller.Tz

        # Start the steering controller if there is a change in the 
        # ball-velocity (dphi) setpoint using the PS4 controller.
        if np.abs(bb_controller.dphi_y_sp) > DPHI_DEADBAND or np.abs(bb_controller.dphi_x_sp) > DPHI_DEADBAND:

            # Find the error for the steering controller PID, using the 
            # "bb_controller.dphi_y_sp" and "bb_controller.dphi_x_sp" variables.

            # Remove the "pass" statement below after implementing the above steps.
            pass

        # Also start the steering controller if the ball-velocity is greater than
        # DPHI_DEADBAND (0.5 rad/sec) to prevent the ball-bot from drifting around
        elif np.abs(dphi_x) > DPHI_DEADBAND or np.abs(dphi_y) > DPHI_DEADBAND:

            # Calculate the errors by setting the velocity setpoint to be zeroes
            # along x and y axes.
            
            # Remove the "pass" statement below after implementing the above steps.
            pass

        # PID - Steering Controller 
        # Your PID implementation goes here 
        # You can implement a velocity based PID or a position based PID for the steering controller.
        # Limit the maximum output of the sum of the balance and steering controllers.
        # Also limit the ratio of the balance and steering controller outputs, if necessary.
        # This is to ensure that the steering controller does not overpower the balance controller.
        
        # If the ball-velocity is less than DPHI_DEADBAND (0.5 rad/sec), 
        # then turn off the steering controller. 
        # Should you reset the error & PID terms too? Think about it.
        else:
            Tx_steer = 0.0
            Ty_steer = 0.0

        # Max Lean angle (Theta) constraint: If theta is greater than the maximum lean angle 
        # (4 degrees), then turn off the steering controller. 
        # Should you reset the error & PID terms too? Think about it.
        if np.abs(theta_x) > MAX_THETA or np.abs(theta_y) > MAX_THETA:
            Tx_steer = 0.0
            Ty_steer = 0.0

        # Summation of planar torques
        # Stability controller + Steering controller
        Tx = Tx_bal + Tx_steer
        Ty = Ty_bal + Ty_steer
        Tz = Tz_bal + Tz_steer

        # Conversion of planar torques to motor torques
        T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)

        # Sending motor torque commands to the pico
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3

        ser_dev.send_topic_data(101, commands)

        if zeroed:
            print(" << Iteration no: {}, DPHI X: {:.2f}, DPHI Y: {:.2f} >>".format(i, dphi[0][0], dphi[1][0]))
            # Construct the data matrix for saving - you can add more variables by replicating the format below
            data = [i] + [t_now] + \
                [states['theta_roll']] + [states['theta_pitch']] + \
                    [Tx] + [Ty] + [Tz] + \
                        [T1] + [T2] + [T3]

            dl.appendData(data)

    print("Resetting Motor commands.")
    time.sleep(0.25)
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    time.sleep(0.25)
    commands['start'] = 0.0
    time.sleep(0.25)
    ser_dev.send_topic_data(101, commands)
    time.sleep(0.25)

    dl.writeOut()
