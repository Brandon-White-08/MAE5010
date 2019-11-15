#!/usr/bin/env python

from __future__ import print_function
from pymavlink import mavutil
import multiprocessing
import numpy as np
import sys
import time
import spidev
import navio2.lsm9ds1
import navio2.ms5611
# import navio2.ublox
import ublox
import navio2.rcinput
import navio2.pwm
import navio2.adc
import navio2.util

current_milli_time = lambda: int(round(time.time() * 1000))


# define indices of y for easier access.
ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
pres_baro = 9
gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d, gps_fix = range(10,17)
adc_a0, adc_a1, adc_a2, adc_a3, adc_a4, adc_a5, est_curr_consumed, last_curr_time = range(17,25)
pres_initial = 25

# define indices of servo for easier access.
mode_flag = 0
rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5 = range(1,7)
servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7,13)
throttle, aileron, elevator, rudder, none, flaps = range(7,13)

# define indices of xh for easier access.
x, y, z, vt, alpha, beta, phi, theta, psi, p, q, r = range(12)


def initialize_gps():
    # ubl = navio2.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
    ubl = ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

    ubl.configure_poll_port()
    ubl.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)

    ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=ublox.PORT_USB, inMask=1, outMask=1)
    ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_poll_port()
    ubl.configure_poll_port(ublox.PORT_SERIAL1)
    ubl.configure_poll_port(ublox.PORT_SERIAL2)
    ubl.configure_poll_port(ublox.PORT_USB)
    ubl.configure_solution_rate(rate_ms=500)
    # this rate can be faster, but there's not much of a performance increase.

    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)

    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_STATUS, 1)
    ubl.configure_message_rate(ublox.CLASS_MON, ublox.MSG_MON_HW, 0)
    ubl.configure_message_rate(ublox.CLASS_MON, ublox.MSG_MON_HW2, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DOP, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SOL, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELECEF, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 0)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 0)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 0)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SVSI, 0)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_ALM, 0)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_EPH, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_TIMEGPS, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_CLOCK, 0)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS, 0)

    for x in range(30):
        msg = ubl.receive_message_noerror()
        # print(msg)

    return ubl

def initialize_sensors():
    print('\ninitializing sensors...')
    adc = navio2.adc.ADC()
    imu = navio2.lsm9ds1.LSM9DS1()
    imu.initialize()
    baro = navio2.ms5611.MS5611()
    baro.test()
    ubl = initialize_gps()
    return adc,imu,baro,ubl

def wait_heartbeat(master):
    print("waiting for heartbeat...")
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print("heartbeat received from {}".format('/dev/ttyAMA0'))
    print(msg.to_dict())
    print()

def send_telemetry(y,xh,servo,master, initial_time):

    # send attitude message #30
    roll = xh[phi]                      # radians (-pi to pi)
    pitch = xh[theta]                       # radians (-pi to pi)
    yaw = xh[psi]                                         # radians (-pi to pi)
    roll_speed = xh[p]                                 # radians / second
    pitch_speed = xh[q]                             # radians / second
    yaw_speed = xh[r]                                  # radians / second
    time_stamp = current_milli_time()-initial_time
    master.mav.attitude_send(time_stamp,roll,pitch,yaw,roll_speed,pitch_speed,yaw_speed)

    # send global_position_int message #33
    # only relative_altitude shows up in mission planner
    latitude = 0                                    # degrees * e7
    longitude = 0                                   # degrees * e7
    altitude = 0                                    # millimeters
    # relative_altitude = y[baro_pressure]/(1.13*9.8065)
    # if (y[pres_initial] != 0): relative_altitude = (10000 - y[pres_baro]/y[pres_initial]*10000)*1000
    # else: relative_altitude = 0
    relative_altitude = -1*xh[z]
    vx = y[gps_vel_n]                                       # centimeters / second
    vy = y[gps_vel_e]   # TEST FOR UNITS                                    # centimeters / second
    vz = y[gps_vel_d]                                       # centimeters / second
    heading = 65535                                 # max value of uint16 implies unknown.
    time_stamp = current_milli_time()-initial_time
    master.mav.global_position_int_send(time_stamp,latitude,longitude,altitude,relative_altitude,vx,vy,vz,heading)

    # send vfr_hud message #74
    # air_speed = 0                                   # meters / second
    # ground_speed = 0#y[gps_groundspeed]            # meters / second
    # heading = 0#y[gps_heading]                     # degrees
    # throttle = 0                                    # percentage
    # altitude = 0                                    # meters
    # climb_rate = 0                                  # meters / second
    # master.mav.vfr_hud_send(air_speed,ground_speed,heading,throttle,altitude,climb_rate)

    # send gps_raw_int message #24
    fix_type=y[gps_fix]
    lat=y[gps_posn_n]              # degrees e7
    long=y[gps_posn_e]            # -970584000
    alt=y[gps_posn_d]              # mm
    eph=65535
    epv=65535
    vel=65535
    cog=65535
    sat_vis=255
    alt_ellipsoid=340000                # mm
    h_acc=0#y[gps_h_acc]               # position uncertainty
    v_acc=0#y[gps_v_acc]
    vel_acc=0#12.0
    hdg_acc=0#12.0
    time_stamp = current_milli_time()-initial_time
    master.mav.gps_raw_int_send(time_stamp,fix_type,lat,long,alt,eph,epv,vel,cog,sat_vis)

    # send sys_status message #1
    # sensors_present = 0
    # sensors_enabled = 0
    # sensors_health = 0
    # load = 0
    # voltage_battery = int(y[adc_a2])
    # current_battery = int(y[adc_a3]/10)
    # battery_remaining = int(y[est_curr_consumed]/5000.*100)  # this is actually % consumed. also inaccurate.
    # # battery_remaining = int(servo[servo_1]*100)-100
    # drop_rate_comm = 0
    # errors_comm = 0
    # master.mav.sys_status_send(sensors_present,sensors_enabled,sensors_health,load,voltage_battery,current_battery,battery_remaining,drop_rate_comm,errors_comm,0,0,0,0)

def read_sensor(y, adc, imu, baro, ubl):

    # update adc data
    for x in range(6):
        y[adc_a0+x] = adc.read(x)    # we don't trust this data at all right now.
    if (y[last_curr_time] != 0.): y[est_curr_consumed] += y[adc_a3]*(current_milli_time()-y[last_curr_time])/3600000.
    y[last_curr_time] = current_milli_time()

    # update imu data
    m9a, m9g, m9m = imu.getMotion9()
    y[ax] = m9a[0]
    y[ay] = m9a[1]
    y[az] = m9a[2]
    y[gyro_p] = m9g[0]
    y[gyro_q] = m9g[1]
    y[gyro_r] = m9g[2]
    y[mag_x] = m9m[0]
    y[mag_y] = m9m[1]
    y[mag_z] = m9m[2]

    # update baro data
    baro.update()
    pressure = baro.returnPressure()
    y[pres_baro] = pressure
    if (y[pres_initial] == 0): y[pres_initial] = pressure

    # update gps data
    p = s = v = False
    while (True):
        msg = ubl.receive_message_noerror()
        if (msg == None and p == s and s == v): break
        if (msg == None): continue
        if msg.name() == "NAV_POSLLH":
            p = True
            msg = (str(msg).split(","))
            y[gps_posn_e] = int(msg[1].split("=")[1])
            y[gps_posn_n] = int(msg[2].split("=")[1])
            y[gps_posn_d] = int(msg[3].split("=")[1])
            # y[gps_h_acc] = int(msg[5].split("=")[1])
            # y[gps_v_acc] = int(msg[6].split("=")[1])
        elif msg.name() == "NAV_STATUS":
            s = True
            y[gps_fix] = int(str(msg).split(",")[1].split("=")[1])
        elif msg.name() == "NAV_VELNED":
            v = True
            msg = (str(msg).split(","))
            y[gps_vel_n] = int(msg[1].split("=")[1])
            y[gps_vel_e] = int(msg[2].split("=")[1])
            y[gps_vel_d] = int(msg[3].split("=")[1])
            # y[gps_groundspeed] = int(str(msg).split(",")[5].split("=")[1])
            # y[gps_heading] = int(str(msg).split(",")[6].split("=")[1])
    return (p and s and v)

def servo_loop(servo):

    rcin = navio2.rcinput.RCInput()

    with navio2.pwm.PWM(0) as pwm_0, navio2.pwm.PWM(1) as pwm_1, navio2.pwm.PWM(2) as pwm_2, navio2.pwm.PWM(3) as pwm_3, navio2.pwm.PWM(5) as pwm_5:#, navio2.pwm.PWM(4) as pwm_4:
        rc_previous = [0.,0.,0.,0.,0.,0.]
        pwm_0.set_period(50)
        pwm_0.enable()
        pwm_1.set_period(50)
        pwm_1.enable()
        pwm_2.set_period(50)
        pwm_2.enable()
        pwm_3.set_period(50)
        pwm_3.enable()
        # pwm_4.set_period(50)
        # pwm_4.enable()
        pwm_5.set_period(50)
        pwm_5.enable()
        while True:
	    initialTime=current_milli_time()
            for x in range(6):
                servo[rcin_0+x] = int(rcin.read(x)) / 1000.0
            if (servo[rcin_4] < 1.250 or servo[rcin_4] > 1.750):
                servo[mode_flag] = 0
                #if (abs(servo[rcin_0] - rc_previous[0]) >= .02): 
		pwm_0.set_duty_cycle(servo[rcin_0])
                #if (abs(servo[rcin_1] - rc_previous[1]) >= .02): 
		pwm_1.set_duty_cycle(servo[rcin_1])
                #if (abs(servo[rcin_2] - rc_previous[2]) >= .02): 
		pwm_2.set_duty_cycle(servo[rcin_2])
                #if (abs(servo[rcin_3] - rc_previous[3]) >= .02): 
		pwm_3.set_duty_cycle(servo[rcin_3])
                # if (abs(servo[rcin_4] - rc_previous[4]) >= .02): pwm_4.set_duty_cycle(servo[rcin_4])
                if (abs(servo[rcin_5] - rc_previous[5]) >= .02): pwm_5.set_duty_cycle(servo[rcin_5])
                for x in range(6):
                    servo[servo_0+x] = servo[rcin_0+x]
                    rc_previous[x] = servo[servo_0+x]
            else:
                servo[mode_flag] = 1
                # set servo pwm using s[servo_0+x] from controller loop.
                s0, s1, s2, s3, s4, s5 = servo[servo_0], servo[servo_1], servo[servo_2], servo[servo_3], servo[servo_4], servo[servo_5]
                #if (abs(s0 - rc_previous[0]) >= .02):
                pwm_0.set_duty_cycle(s0)
                    #rc_previous[0] = s0
                #if (abs(s1 - rc_previous[1]) >= .02):
                pwm_1.set_duty_cycle(s1)
                    #rc_previous[1] = s1
                #if (abs(s2 - rc_previous[2]) >= .02):
                pwm_2.set_duty_cycle(s2)
                    #rc_previous[2] = s2
                #if (abs(s3 - rc_previous[3]) >= .02):
                pwm_3.set_duty_cycle(s3)
                    #rc_previous[3] = s3
                # if (abs(s4 - rc_previous[4]) >= .02):
                # pwm_4.set_duty_cycle(s4)
                    # rc_previous[4] = s4
                #if (abs(s5 - rc_previous[5]) >= .02):
                pwm_5.set_duty_cycle(s5)
                    #rc_previous[5] = s5
            time.sleep(0.005-(current_milli_time()-initialTime)/1000)
    
def telemetry_loop(y,xh,servo,master):

    wait_heartbeat(master)
    initial_time = current_milli_time()

    time_sent = 0
    frequency = 3.0
    period = 1000.0 / frequency

    while True:
        if ((current_milli_time() - time_sent + 50) >= period):
            send_telemetry(y,xh,servo,master,initial_time)
            time_sent = current_milli_time()
            # print("telemetry sent.")
