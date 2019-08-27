#					TITLE BLOCK
#**************************************************
#Author:	Brandon White
#Date:		08/26/2019
#Desc:		This set of functions to solve the problems
#			given on HW1 of MAE 5010.
#**************************************************

def Euler3212EP(angles, radians = False, rounding = True):
	#Expects degrees but can accept radians with flag set

	import math

	#convert to radians for math
	if not radians:
		for i in range(3):
			angles[i] = angles[i] * (math.pi/180)

	[psi, theta, phi] = angles

	e = [math.cos(psi/2)*math.cos(theta/2)*math.cos(phi/2) + math.sin(psi/2)*math.sin(theta/2)*math.sin(phi/2),
			math.cos(psi/2)*math.cos(theta/2)*math.sin(phi/2) - math.sin(psi/2)*math.sin(theta/2)*math.cos(phi/2),
			math.cos(psi/2)*math.sin(theta/2)*math.cos(phi/2) + math.sin(psi/2)*math.cos(theta/2)*math.sin(phi/2),
			math.sin(psi/2)*math.cos(theta/2)*math.cos(phi/2) - math.cos(psi/2)*math.sin(theta/2)*math.sin(phi/2)]

	if rounding:
		for i in range(4):
			e[i] = round(e[i],5)

	return e

def EP2Euler321(e, rounding = True):
	import math

	angles = [math.atan2(2 * (e[0]*e[1] + e[2]*e[3]), e[0]**2 + e[3]**2 - e[1]**2 - e[2]**2),
				math.asin(2 * (e[0]*e[2] - e[1]*e[3])),
				math.atan2(2 * (e[0]*e[3] + e[2]*e[1]), e[0]**2 + e[1]**2 - e[2]**2 - e[3]**2)]

	#Convert to degrees
	for i in range(3):
		angles[i] = angles[i] * (180/math.pi)
		if rounding:
			angles[i] = round(angles[i],2)

	return angles #returns degrees

def make_gamma(I):
	[Ixz, Ix, Iy, Iz] = I
	Gamma = [Ix*Iz - Ixx**2, 0, 0, 0, 0, 0, 0, 0, 0]
	Gamma[1] = Ixz*(Ix-Iy-Iz)/Gamma[0]
	Gamma[2] = (Iz*(Iz-Iy)+Ixz**2)/Gamma[0]
	Gamma[3] = Iz/Gamma[0]
	Gamma[4] = Ixz/Gamma[0]
	Gamma[5] = (Iz - Ix)/Iy
	Gamma[6] = Ixz/Iy
	Gamma[7] = (Ix*(Ix-Iy)+Ixz**2)/Gamma[0]
	Gamma[8] = Ix/Gamma[0]
	return Gamma

def pos_kin(psi, theta, phi):
	#d/dt([p_n, p_e, p_d])
	from numpy import matmul

	A1 = [[cos(theta)*cos(psi), sin(phi)*sin(theta)*sin(psi) - cos(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)],
				[cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)],
				[-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]]
	b1 = [u, v, w]
	return matmul(A1, b1)

def pos_dyn(u, v, w):
	#d/dt([u, v, w])
	x_dot = [[r*v-q*w + Fx/m],
			[p*w-r*u + Fy/m],
			[q*u-p*v + Fz/m]]
	return x_dot

def rot_kin(e0, e1, e2, e3):
	#d/dt( e )
	from numpy import matmul
	A3 = [[0, -p/2, -q/2, -r/2],
			[p/2, 0, r/2, -q/2],
			[q/2, -r/2, 0, p/2],
			[r/2, q/2, -p/2, 0]]
	b3 = [e0, e1, e2, e3]
	return matmul(A3, b3)

def rot_dyn(Gamma):
	#d/dt([p, q, r])
	x_dot = [[Gamma[1]*p*q - Gamma[2]*q*r + Gamma[3]*L + Gamma[4]*N],
			[Gamma[5]*p*r - Gamma[6]*(p**2-r**2) + 1/Iy*M],
			[Gamma[7]*p*q - Gamma[1]*q*r + Gamma[4]*L + Gamma[8]*N]]
	return x_dot

def derivatives(state, FM, MAV):
	#state:	[p_n, p_e, p_d, u, v, w, e0, e1, e2, e3, p, q, r]
	#FM:	[Fx, Fy, Fz, Ell, M, N]
	#MAV:	MAV.inertnert, MAV.m, MAV.gravity_needed

	from math import sin, cos
	import EP2Euler321

	#Unpack state, FM, MAV
	[p_n, p_e, p_d, u, v, w, e0, e1, e2, e3, p, q, r] = state
	[Fx, Fy, Fz, L, M, N] = FM
	[Ixz, Ix, Iy, Iz] = MAV.inert

	#Add gravity if not included
	if MAV.gravity_needed:
		Fz = Fz - 32.2*MAV.mass

	#Get angle measures
	angles = EP2Euler321([e0, e1, e2, e3])
	[psi, theta, phi] = angles

	#Get Xdot Terms
	d_dt = [[],[],[]]
	d_dt[0] = pos_kin(psi, theta, phi)
	d_dt[1] = pos_dyn(u,v,w)
	d_dt[2] = rot_kin(e0, e1, e2, e3)
	d_dt[3] = rot_dyn(make_gamma(MAV.inert))

	#Build One Vector of Xdot
	for eqn_set in d_dt:
		for dot in eqn_set:
			xdot.append(dot)

	return xdot

def integrator(tf = 10, delta_t = .25):
