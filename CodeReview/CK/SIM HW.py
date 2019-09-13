#======================================================================
# NAME: Cole Kelly
# TITLE: MAE5010 Homework 1                             -,-'-<@
# DATE: 8/29/2019
#======================================================================

from math import *
from numpy import matmul
from numpy import linspace
from scipy.integrate import odeint


class MAV:
    def __init__(self, name, Ix, Iy, Iz, Ixz, mass):
        self.name = name
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz
        self.Ixz = Ixz
        self.mass = mass
        self.u = 0
        self.v = 0
        self.w = 0
        self.p = 0
        self.q = 0
        self.r = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0
        self.pn = 0
        self.pe = 0
        self.pd = 0


    def introduce_self(self):
        print("Aircraft Name: " + self.name)
        print("Aircraft Mass: " + str(self.mass) + " slugs")
        print("Ix = " + str(self.Ix) + " (slugs*ft^2)")
        print("Iy = " + str(self.Iy))
        print("Iz = " + str(self.Iz))
        print("Ixz = " + str(self.Ixz))

    def inertias(self):
        return [self.Ix, self.Iy, self.Iz, self.Ixz, self.mass]
    def store_inertias(self, inertias):
        [self.Ix, self.Iy, self.Iz, self.Ixz] = inertias
    def store_velocities(self, tran_vel, rot_vel):
        [self.u, self.v, self.w] = tran_vel
        [self.p, self.q, self.r] = rot_vel
    def store_position(self, p_vals):
        [self.pn, self.pe, self.pd] = p_vals


def Euler3212EP(angles, rad = True):
    #angles = [heading, pitch, roll] and return EP = [q0, q1, q2, q3]'

    [phi, theta, psi] = angles

    if not rad:
        for x in range(3):
            angles[x] = angles[x] * pi / 180


    EP = [cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2),
          cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2),
          cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2),
          sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2)]

    for x in range(4):
        EP[x] = round(EP[x],4)

    return EP
def EP2Euler321(EP):
    #manipulates q matrix, q=[q0, q1, q2, q3] and returns EA = [heading, pitch, roll]'

    angles = [atan2(2*(EP[0]*EP[1]+EP[2]*EP[3]),(EP[0]**2+EP[3]**2-EP[1]**2-EP[2]**2)),
              asin(2*(EP[0]*EP[2]-EP[1]*EP[3])),
              atan2(2*(EP[0]*EP[3]+EP[1]*EP[2]),(EP[0]**2+EP[1]**2-EP[2]**2-EP[3]**2))]

    return angles

def gamma_Matrix(inertias):
    [Ix, Iy, Iz, Ixz] = inertias

    Gamma = [Ix * Iz - Ixz ** 2, 0, 0, 0, 0, 0, 0, 0, 0]
    Gamma[1] = Ixz * (Ix - Iy - Iz) / Gamma[0]
    Gamma[2] = (Iz * (Iz - Iy) + Ixz ** 2) / Gamma[0]
    Gamma[3] = Iz / Gamma[0]
    Gamma[4] = Ixz / Gamma[0]
    Gamma[5] = (Iz - Ix) / Iy
    Gamma[6] = Ixz / Iy
    Gamma[7] = (Ix * (Ix - Iy) + Ixz ** 2) / Gamma[0]
    Gamma[8] = Ix / Gamma[0]

    print("***Gamma Matrix created***")
    return Gamma
def rotational_kinematics(e_array, rot_vel):
    [p, q, r] = rot_vel

    C = [[0, -p, -q, -r],
         [p, 0, r, -q],
         [q, -r, 0, p],
         [r, q, -p, 0]]

    D = e_array
    rotkin = 0.5 * matmul(C, D)

    print("***Rotational Kinematics Matrix Created***")
    return rotkin
def rotational_dynamics(rot_vel, inertias, moments):
    [p, q, r] = rot_vel
    [M, N, L] = moments
    Gamma = gamma_Matrix(inertias)

    rotdyn = [Gamma[1] * p * q - Gamma[2] * q * r + Gamma[3] * L + Gamma[4] * N,
              Gamma[5] * p * r - Gamma[6] * (p ** 2 - r ** 2) + (1 / Iy) * M,
              Gamma[7] * p * q - Gamma[1] * q * r + Gamma[4] * L + Gamma[8] * N]

    return rotdyn
def positional_kinematics(angles, tran_vel):
    [u, v, w] = tran_vel
    [phi, theta, psi] = angles

    A = [[cos(theta) * cos(psi), sin(phi) * sin(theta) * sin(psi) - cos(phi) * cos(psi),
          cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)],
         [cos(theta) * sin(psi), sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi),
          cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)],
         [-sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta)]]
    B = [u, v, w]
    poskin = matmul(A, B)

    return poskin
def positional_dynamics(rot_vel, tran_vel, forces, m):
    [u, v, w] = tran_vel
    [p, q, r] = rot_vel
    [Fx, Fy, Fz] = forces

    posdyn = [r * v - q * w + Fx / m,
              p * w - r * u + Fy / m,
              q * u - p * v + Fz / m]

    return posdyn

def force_calc(angles, force_input, mass):
    Fg = [0, 0, 32.2]
    [phi, theta, psi] = angles
    [fx, fy, fz] = force_input

    Rbf = [[cos(psi) * cos(theta), sin(psi) * cos(theta), -sin(theta)],
           [cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi),
            cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi), cos(theta) * sin(phi)],
           [sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi),
            sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi), cos(theta) * cos(phi)]]

    [Fxg, Fyg, Fzg] = matmul(Rbf, Fg)

    forces = [Fxg+fx, Fyg+fy, Fzg*mass+fz]
    return forces

def derivatives(MAV, state, FM, t):
    [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r] = state
    [fx, fy, fz, M, N, L] = FM
    [Ix, Iy, Iz, Ixz, mass] = MAV.Inertias()
    [phi, theta, psi] = EP2Euler321([e0, e1, e2, e3])

    time = t



    angles = [phi, theta, psi]
    inertias = [Ix, Iy, Iz, Ixz]
    tran_vel = [u, v, w]
    rot_vel = [p, q, r]
    moments = [L, M, N]
    force_input = [fx, fy, fz]
    forces = force_calc(angles, force_input, mass)

    poskin = positional_kinematics(angles, tran_vel)
    posdyn = positional_dynamics(rot_vel, tran_vel, forces, mass)
    rotkin = rotational_kinematics([e0, e1, e2, e3], rot_vel)
    rotdyn = rotational_dynamics(rot_vel, inertias, moments)



    #store values in object
    MAV.store_velocities(tran_vel, rot_vel)
    MAV.store_position(pn, pe, pd)



    # CALCULATING DERIVATIVES
    # position kinematics
    pn_dot = poskin[0]
    pe_dot = poskin[1]
    pd_dot = poskin[2]

    # position dynamics
    u_dot = posdyn[0]
    v_dot = posdyn[1]
    w_dot = posdyn[2]

    # rotational kinematics
    e0_dot = rotkin[1]
    e1_dot = rotkin[1]
    e2_dot = rotkin[2]
    e3_dot = rotkin[3]

    # rotational dynamics
    p_dot = rotdyn[0]
    q_dot = rotdyn[1]
    r_dot = rotdyn[2]

    xdot = [pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]
    return xdot

def integrator(FM, init_pos, init_vels, init_angles, MAV, tf, deltat, graphing = True):
    pts = round(tf/deltat)
    val = pts+1
    t = linspace(0, tf, val)

    [u, v, w, p, q, r] = init_vels
    [pn, pe, pd] = init_pos
    [e0, e1, e2, e3] = Euler3212EP(init_angles)

    init_state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]

    func = odeint(derivatives, init_state, t, args=(MAV, init_state, FM, ))      #<<<<<<<<<<<<<<<<<<HELP

    if graphing:                                                                #<<<<<<<<<<<<<<<<<<HELP
        from mpl_toolkits import mplot3d
        import matplotlib.pyplot as plt

        figure = plt.figure()
        XX = plt.axes(projection = "3d")
        XX.mplot3D(func[:,0], func[:,1], func[:,2], linestyle = '--', marker = '.')
        XX.set_xlabel('X axis')
        XX.set_ylabel('Y axis')
        XX.set_zlabel('Z axis')
        plt.show()


#RUN
f16 = MAV("F-16 Fighting Falcon", 9496, 55814, 63100, 982, 1491)
f16.introduce_self()

init_vel = [500, 0, 0, 0, 0, 0]    # u , v , w , p , q , r
init_pos = [0, 0, -500]             # x , y , z (neg z = pos altitude)
init_angles = [0, 0, 0]             # phi , theta , psi (rad)
FM = [500, 0, 0, 0, 0, 0]           # fx , fy , fz , M , N , L
state = [0, 0, -500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # starting at altitude of 500ft, no other state variables
tf = 10             #sec
delta_t = 0.5       #sec

integrator(FM, init_pos, init_vel, init_angles, f16, tf, delta_t)