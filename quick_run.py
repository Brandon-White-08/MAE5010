import integrator
from MAV import MAV
import state_viewer
import matplotlib.pyplot as plt

my_mav = MAV('hw1_1')
my_mav.state0
my_mav.FMeq
[t,s] = integrator.integrator(my_mav)


state_viewer.open_GUI(t, s)
