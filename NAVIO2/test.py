#!/usr/bin/env python
# MAE 5010 Autopilot Design airborne man test file.
# 3 Nov 2019, N. Baker and I. Faruque, i.faruque@okstate.edu

from __future__ import print_function
from pymavlink import mavutil
import multiprocessing, time
import numpy as np
import air
import math

current_milli_time = lambda: int(round(time.time() * 1000))

# define indices of xh for easier access.
x, y, z, vt, alpha, beta, phi, theta, psi, p, q, r = range(12)

# define indices of y for easier access.
ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
pres_baro = 9
gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d = range(10, 16)

# define indices of servo for easier access.
mode_flag = 0
rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5 = range(1, 7)
servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7, 13)
throttle, aileron, elevator, rudder, none, flaps = range(7, 13)

# define indices of cmd for easier access.
psi_c, h_c = range(2)


def estimator_loop(y, xh, servo):
    # get sensors for read_sensor function call.
    adc, imu, baro, ubl = air.initialize_sensors()

    time.sleep(3)

    while True:
        new_gps = air.read_sensor(y, adc, imu, baro, ubl)  # updates values in y
        # print(new_gps)
        if (new_gps):
            pass  # do estimation with gps here.
        else:
            pass  # do estimation without gps here.
            y = np.array(
                [0.538385085, -0.035892339, 9.662217659, -0.002443459, 0.133168509, 0.014660753, -5.046, -20.184,
                 57.246])
            A = [[-0.028, 0.233, 0, -9.815, 0, 0, 0, 0, 0, 0, 0, 0]
                 [-0.978, -8.966, 20.1170, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                 [0.102, 0.022, -6.102, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                 [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                 [0, 0, 0, 0, -0.45, 0, -0.986, 0.635, 0, 0, 0, 0]
                 [0, 0, 0, 0, 57.028, -72.97, 3.279, 0, 0, 0, 0, 0]
                 [0, 0, 0, 0, 135.737, -0.588, -4.436, 0, 0, 0, 0, 0]
                 [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                 [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                 [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

            B = [[0, 1, 0, 0]
                 [-23.448, 0, 0, 0]
                 [-50.313, -0.104, 0, 0]
                 [0, 0, 0, 0]
                 [0, 0, 0, 0.315]
                 [0, 0, 677.27, 18.099]
                 [0, 0, -8.875, -99.521]
                 [0, 0, 0, 0]
                 [0, 0, 0, 0]
                 [0, 0, 0, 0]
                 [0, 0, 0, 0]
                 [0, 0, 0, 0]]

            F = np.expm(A*0.01)

            def heading_calc(mag_x, mag_y, mag_z, phi, theta):
                dt = 3  # declination angle in degrees
                M_B = np.array([mag_x, mag_y, mag_z])  # magnetometer output
                M_i = Rot_i_B(M_B, phi, theta)  # put magnetometer in inertial frame
                psi = dt + math.atan(M_i[1] / M_i[2])  # dt is declination angle
                return psi

            def Rot_i_B(Matrix, phi, theta):
                Rot_Mat = np.array([[math.cos(theta), math.sin(theta) * math.sin(phi), math.sin(theta) * math.cos(phi)],
                                    [0, math.cos(phi), -math.sin(phi)],
                                    [-math.sin(theta), math.cos(theta) * math.sin(phi),
                                     math.cos(theta) * math.cos(phi)]])

                R_Mat = Matrix * Rot_Mat
                return R_Mat

            # define initial states from y
            ax = y[0]
            ay = y[1]
            az = y[2]
            gyro_p = y[3]
            gyro_q = y[4]
            gyro_r = y[5]
            mag_x = y[6]
            mag_y = y[7]
            mag_z = y[8]

            # definition of noise and biases
            AccelVariance = .002  # noise of accelerometer
            GyroVariance = 1e-5  # noise of gyro
            AttitudeVariance = .3  # attitude noise
            AccelBias = np.array([0, 0, 0])  # Bias of Accelerometer... from christian
            GyroBias = np.array([0, 0, 0])  # gyro bias
            MagBias = np.array([0, 0, 0])  # magnetometer bias

            # initial orientation estimate
            phi = 0
            theta = 0
            psi = heading_calc(mag_x, mag_y, mag_z, phi, theta)  # need to create function to do this
            euler = np.array([phi, theta, psi])

            # define x_hat
            x_hat = np.array(
                [euler, GyroBias, AccelBias])  # 1-3 Euler angles ,,,, #4-6 xyz gyro bias estimates, #7-9 xyz gyro bias

            # define R uncertainty in measurement
            Variance = np.arrays[AccelVariance,AccelVariance,AccelVariance,GyroVariance,GyroVariance,GyroVariance,AttitudeVariance,AttitudeVariance,AttitudeVariance]
            Identity = np.identity(9)
            R = Identity * Variance

            # Define Q uncertainty in model
            Q = np.identity(9)

            EKFl = EKF(initial_x=x_hat, initial_P=P)
            test = EXF1.step(F, Q, G, U, y, hx, C, R)

        # write estimated values to the xh array.


def controller_loop(xh, servo, cmd):
    while True:
        if (servo[mode_flag] == 1):
            pass  # rewrite servo_out values to servo array based on their previous values and xh.
            # if (servo[servo_1]<1.5): servo[servo_1] = 1.55
            # else: servo[servo_1] = 1.45
            # time.sleep(1)
            # Controller should assign values in range 1.25 to 1.75 to outputs;
            # WARNING, servo damage likely if values outside this range are assigned
            # Example: This is a manual passthrough function
            servo[throttle] = servo[rcin_0]
            servo[aileron] = servo[rcin_1]
            servo[elevator] = servo[rcin_2]
            servo[rudder] = servo[rcin_3]
            servo[servo_4] = servo[servo_4]  # no servo; channel used for manual/auto switch
            servo[flaps] = servo[rcin_5]


if __name__ == "__main__":

    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600, source_system=255)

    # initialize arrays for sharing sensor data.
    y = multiprocessing.Array('d', np.zeros(26))  # imu, baro, gps, adc
    xh = multiprocessing.Array('d', np.zeros(12))  # position, orientation, rates
    servo = multiprocessing.Array('d', np.zeros(13))  # mode_flag, rcin, servo_out
    cmd = multiprocessing.Array('d', np.zeros(2))  # psi_c, h_c

    # start processes for interpreting sensor data and setting servo pwm.
    estimator_process = multiprocessing.Process(target=estimator_loop, args=(y, xh, servo))
    estimator_process.daemon = True
    estimator_process.start()
    controller_process = multiprocessing.Process(target=controller_loop, args=(xh, servo, cmd))
    controller_process.daemon = True
    controller_process.start()
    servo_process = multiprocessing.Process(target=air.servo_loop, args=(servo,))
    servo_process.daemon = True
    servo_process.start()
    time.sleep(5)
    # start process for telemetry after other processes have initialized.
    telemetry_process = multiprocessing.Process(target=air.telemetry_loop, args=(y, xh, servo, master))
    telemetry_process.daemon = True
    telemetry_process.start()

    print("\nsending heartbeats to {} at 1hz.".format('/dev/ttyAMA0'))
    # loop for sending heartbeats and receiving messages from gcs.
    while True:
        time_sent = 0
        while True:
            # send heartbeat message if one second has passed.
            if ((current_milli_time() - time_sent) >= 980):
                master.mav.heartbeat_send(1, 0, 0, 0, 4, 0)
                # still haven't figured out how to get mode to show up in mission planner.
                # print('heartbeat sent.')
                time_sent = current_milli_time()
            # Simple waypoint tracker
            # DO WAYPOINT TRACKING HERE

            # handle incoming commands over telemetry
            # try:
            #     msg = master.recv_match().to_dict()
            #     if (not (msg['mavpackettype'] == 'RADIO' or msg['mavpackettype'] == 'RADIO_STATUS' or msg['mavpackettype'] == 'HEARTBEAT')):
            #         print(msg)
            #         if (msg['mavpackettype'] == 'COMMAND_LONG'):
            #             master.mav.command_ack_send(msg['command'],4)
            #             print("acknowledge sent.")
            # except:
            #     pass