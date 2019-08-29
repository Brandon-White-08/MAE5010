import white_brandon_HW1 as HW1
from MAV import MAV
import state_viewer
my_mav = MAV()
my_mav.state0
my_mav.FMeq
[t,s] = HW1.integrator(my_mav)
state_viewer.open_GUI(t, s)
