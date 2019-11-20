#Notes
# EQUATIONS
# ax = u_dot + p*w - v*r + g*sin(theta)
# ay = v_dot + r*u - w*q + g*cos(theta)*sin(phi)
# az = w_dot + p*w - q*r + g*cos(theta)*cos(phi)

def eliminate_bias():
    from time import time, sleep
    import numpy as np

    print('WARNING! LEVEL AIRCRAFT UNTIL FURTHER NOTICE!')
    #Give 30 seconds of warmup
    t1 = time()
    gyro = np.array([0 0 0])
    while time()-t1 < 30
        gyro = np.append(gyro, NEW_MEASUREMENTS, axis = 1)
        sleep(0.5)

    bias = [np.average(gyro[:,0]), np.average(gyro[:,1]), np.average(gyro[:,2])]

    print('Sensor Bias Calibration Completed')

    return bias

def estimator():
    from math import *
    from numpy import *
    from time import time,sleep

    #Given ax ay az as row vector
    accel = imu_read()

    #Correct directionalities
    R_imu = array([[0 -1 0], [-1 0 0], [0 0 1]])
    [ax, ay, az][0] = dot(R_imu, transpose(accel))

    #Completing angular relations
    phi_a = atan2(ay / sqrt(ax**2 + az**2))     # Reliable
    theta_a = atan2(ay / sqrt(ax**2 + az**2))   # Reliable
    psi_a = atan2(ay / sqrt(ax**2 + az**2))     # Unreliable

    #Now that we have these values, we make a rotation matrix
    Rhb = array([[],
                 [],
                 []])

    #Accept the rate gyro values
    p = 0
    q = 0
    r = 0

    if new_gps:
        [u_old, v_old, w_old] = [u, v, w]
        [x, y, z, u, v, w] = gps_read()
        delta_t = round(time() - t1, 3)
        t1 = time()
        [u_dot v_dot w_dot] = ([u, v, w] - [u_old, v_old, w_old]) / delta_t

        z = barometer_read()

    psi = 0

    xh = array([x, y, z, u, v, w, p, q, r, phi, theta, psi])

    return xh
