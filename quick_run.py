import integrator
from MAV import MAV
import state_viewer
import matplotlib.pyplot as plt

def main():
    my_mav = MAV('hw2')
    my_mav.state0
    my_mav.FMeq
    [t,s] = integrator.integrator(my_mav, tf = 0.1, delta_t = .01)

    state_viewer.open_GUI(t, s)
    return [t,s]

main()
