import white_brandon_HW1 as HW1
from MAV import MAV
import state_viewer
import matplotlib.pyplot as plt
[t,s] = HW1.integrator(MAV())

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
fig.suptitle('quaternions')
ax1.plot(t,s[:,6])
ax2.plot(t,s[:,7], 'tab:orange')
ax3.plot(t,s[:,8], 'tab:green')
ax4.plot(t,s[:,9], 'tab:red')
for ax in fig.get_axes():
    ax.label_outer()
plt.show()

state_viewer.open_GUI(t, s)
