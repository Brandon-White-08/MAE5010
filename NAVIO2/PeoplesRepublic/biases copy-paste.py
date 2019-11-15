
#servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7, 13)
#throttle, aileron, elevator, rudder, none, flaps = range(7, 13)

# define indices of cmd for easier access.
#psi_c, h_c = range(2)

class bias(object): #creates an object that avgs bias for 25 inputs--Christian Coletti
    def __init__(self):
        self.avg=0
        self.test=0

    def add(self, val):
        self.avg=self.avg + val/25
        self.test=self.test + 1/25


avyax = bias()
avyay = bias()
avyaz = bias()
avygyp = bias()
avygyq = bias()
avygyr = bias()
avybaro = bias()
avy_gpsvn = bias()
avy_gpsve = bias()
avy_gpsvd = bias()


def avgs(y):
    avyax.add(y[ax])
    avyay.add(y[ay])
    avyaz.add(y[az])
    avygyp.add(y[gyro_p])
    avygyq.add(y[gyro_q])
    avygyr.add(y[gyro_r])

def avgsgps(y):
    avybaro.add(y[pres_baro])
    avy_gpsvn.add(gps_vel_n)
    avy_gpsve.add(gps_vel_e)
    avy_gpsvd.add(gps_vel_d)

i = 0
j = 0

def estimator_loop(y):
    # get sensors for read_sensor function call.
#    adc, imu, baro, ubl = air.initialize_sensors()
    time.sleep(3)

    while True:
        global i, j
        initialTime = current_milli_time()
        new_gps = air.read_sensor(y, adc, imu, baro, ubl)  # updates values in y
        if i < 25:
            i = i + 1
            avgs(y)
        if j < 25 and new_gps == True:
            j = j + 1
            avgsgps(y)
        elif j < 26:
            j = j+1

        # print(new_gps)
        #if (new_gps):
        #    pass  # do estimation with gps here.
