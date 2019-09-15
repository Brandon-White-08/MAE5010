import integrator
from MAV import MAV
import state_viewer
import matplotlib.pyplot as plt

def main():
    my_mav = MAV('hw2')
    my_mav.controls[0]=-.785 #45%
    my_mav.controls[1]= .80   # % max thrust_max
    my_mav.state0[3]=50
    my_mav.state0
    my_mav.FMeq
    [t,s] = integrator.integrator(my_mav, tf = 20, delta_t = .1)
    txtfile = open("output.txt", "w+")
    for state in s:
        for item in state:
            txtfile.write(str(item))
            txtfile.write(",")
        #txtfile.write('\n')
    txtfile.close()

    state_viewer.open_GUI(t, s)




main()
