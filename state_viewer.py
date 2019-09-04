#					TITLE BLOCK
#**************************************************
#Author:	Brandon White
#Date:		08/28/2019
#Desc:		Creates a visual representation of
#			states vs time for sim data
#**************************************************

#Required imported modules:
#	matplotlib (python -m pip install matplotlib)
#	PyQt5 (python -m pip install pyqt5)
#	numpy (python -m pip install numpy)
#	sys (installed by default)
#	time (installed by default)

import sys

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QWidget, QPushButton
from PyQt5.QtGui import QIcon

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import numpy, time

#Main GUI Class
class App(QMainWindow):

	def __init__(self):
		super().__init__()

		#Set form size (NOTE: graphs are [Pixels Width, Pixels Length] / 100)
		self.width = 1540 #40 for gaps
		self.height = 700 #50 for gapsS

		#GUI Position and Size
		self.left = 0
		self.top = 55
		self.title = 'Simulation Data Visualizer'
		self.graphs = []


	def initUI(self):
		self.setWindowTitle(self.title)
		self.setGeometry(self.left, self.top, self.width, self.height)

		#Create all graphs
		self.graphs.append(PlotCanvas(self, width=5, height=5,
			name_here = "position", given_data = self.data[:,0:3], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.6,
			name_here = "u", given_data = self.data[:, 3], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.6,
			name_here = "v", given_data = self.data[:, 4], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.6,
			name_here = "w", given_data = self.data[:, 5], t=self.t))
		self.graphs.append(PlotCanvas(self, width=3.3, height=1.8,
			name_here =  "p", given_data = self.data[:, 10], t=self.t))
		self.graphs.append(PlotCanvas(self, width=3.3, height=1.8,
			name_here = "q", given_data = self.data[:, 11], t=self.t))
		self.graphs.append(PlotCanvas(self, width=3.3, height=1.8,
			name_here = "r", given_data = self.data[:, 12], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.6,
			name_here = "q0", given_data = self.data[:, 6], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.6,
			name_here = "q1", given_data = self.data[:, 7], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.6,
			name_here = "q2", given_data = self.data[:, 8], t=self.t))
		self.graphs.append(PlotCanvas(self, width=5, height=1.8,
			name_here = "q3", given_data = self.data[:, 9], t=self.t))

		#Position Graphs
		self.graphs[0].move(10,0)
		self.graphs[1].move(520,0)
		self.graphs[2].move(520,170)
		self.graphs[3].move(520,340)
		self.graphs[4].move(10,510)
		self.graphs[5].move(350,510)
		self.graphs[6].move(690,510)
		self.graphs[7].move(1030,0)
		self.graphs[8].move(1030,170)
		self.graphs[9].move(1030,340)
		self.graphs[10].move(1030,510)

		for graph in self.graphs:
			graph.plot()

		self.show()

# Graphing Subclass
class PlotCanvas(FigureCanvas):

	def __init__(self, parent=None, width=5, height=4, dpi=100, name_here = "NO NAME GIVEN", given_data = [0], t=[0]):
		fig = Figure(figsize=(width, height), dpi=dpi)
		self.axes = fig.add_subplot(111)

		self.nombre = name_here
		self.t = t
		self.data = given_data

		FigureCanvas.__init__(self, fig)
		self.setParent(parent)

		FigureCanvas.setSizePolicy(self,
				QSizePolicy.Expanding,
				QSizePolicy.Expanding)
		FigureCanvas.updateGeometry(self)
		self.plot()

	def plot(self):
		try:
			if self.nombre == "position":
				import matplotlib.pyplot as plt
				from mpl_toolkits.mplot3d import Axes3D
				ax = self.figure.add_subplot(111, projection = '3d')
				ax.cla()
				ax.plot3D(self.data[:, 0], self.data[:, 1], self.data[:, 2])
				ax.set_xlabel('P_n')
				ax.set_ylabel('P_e')
				ax.set_zlabel('P_d')
				ax.set_title(self.nombre)
				ax.invert_zaxis()
				self.draw()
			elif self.nombre == 'u' or self.nombre == "v" or self.nombre == "w":
				ax = self.figure.add_subplot(111)
				ax.plot(self.t,self.data, 'r')
				ax.set_title(self.nombre + ' (ft/s vs s)')
				a = numpy.amin(self.data)
				b = numpy.amax(self.data)
				ax.set_yticks([a, a+(b-a)/4, a+2*(b-a)/4, a+3*(b-a)/4, b])
				ax.set_ylabel('ft/s')
				self.draw()
			elif self.nombre == 'p' or self.nombre == "q" or self.nombre == "r":
				ax = self.figure.add_subplot(111)
				ax.plot(self.t,self.data, 'r')
				ax.set_title(self.nombre + ' (rad/s vs s)')
				ax.set_ylabel('rad/s')
				a = numpy.amin(self.data)
				b = numpy.amax(self.data)
				ax.set_yticks([a, a+(b-a)/4, a+2*(b-a)/4, a+3*(b-a)/4, b])
				self.draw()
			else:
				ax = self.figure.add_subplot(111)
				ax.plot(self.t,self.data, 'r')
				ax.set_title(self.nombre)
				ax.set_xlabel('t (s)')
				ax.set_ylabel('mag.')
				ax.set_ylim(-1,1)
				self.draw()
		except:
			print('PLOT METHOD ERROR')
			return

def open_GUI(t = [0], sim_data = [0,0,0,0,0,0,0,0,0,0,0,0,0]):
	app = QApplication(sys.argv)
	ex = App()
	ex.data = sim_data
	ex.t = t
	ex.initUI()
	sys.exit(app.exec_())

if __name__ == '__main__':
	open_GUI()
