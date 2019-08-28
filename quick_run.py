import white_brandon_HW1 as HW1
from MAV import MAV
import state_viewer
[t,s] = HW1.integrator(MAV())
state_viewer.open_GUI(t, s)
