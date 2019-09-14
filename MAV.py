#					TITLE BLOCK
#**************************************************
#Author:	Brandon White
#Date:		08/26/2019
#Desc:		Creates a MAV object with mass, moment
#           of inertia, and gravity properties
#**************************************************

from rotations import *

#Calling the class with an aircraft name below creates an MAV object
class MAV:
    def __init__(self, aircraft = "None"):
       #All units listed in English units as denoted
       #NOTE: alpha and beta in radians
       self.name = aircraft
       self.mass = 10  # Mass (slug)
       self.dynamic_density = False #True uses lapse rate for rho=f(h)

       #Inert = [Ixz, Ix, Iy, Iz]
       self.inert = [20, 10, 10, 10]  # Moment of Inertia (lbf*ft^2)

       #State = [p_n, p_e, p_d, u, v, w, e0, e1, e2, e3, p, q, r]
       self.state0 = [0, 0, -500, 50, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            #Level flight at 500 ft at 50 ft/s

       #FM = [Fx, Fy, Fz, Ell, M, N]
       self.FM = [0, 0, 0, 0, 0, 0]
            #Equations for time-variant forces and moments
       self.FMeq = [0, 0, 0, 0, 0, 0]

       self.thrust_max = 10  #lbf

       #Geometric Properties and Coefficients
        #Order: b, c, x_cg, y_cg, z_cg, stall, coefficients
            #stall: True/False, M, alpha_0
            #coeff: CL0, CLa/b, CLq, CL_del_control, CD0, CDa/b, CDq, CD_del_control, [spare]
       self.wing = [0, 0, 0, 0, 0, [True, 50, 0.471],
                    [0, 0, 0, 0, 0, 0, 0, 0, [0,0,0]]]
       self.hstab = [0, 0, 0, 0, 0, [False, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, [0,0,0]]]
       self.vstab = [0, 0, 0, 0, 0, [False, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, [0,0,0]]]

       #Controls Deflections: del_e, del_t(<1), del_a, del_r
       self.controls = [ 0, 1, 0, 0]

       #self.coeffeq = [0, 0, 0, 0, 0, 0, 0, 0,
        #                0, 0, 0, 0, 0, 0, 0, 0,
        #                0, 0, 0, 0, 0, 0, 0, 0]

       if aircraft != "None":
           try:
               method_to_call = getattr(self, aircraft.lower())
               method_to_call()
           except:
               print("No preconfig by given name: " + aircraft.lower())

    def density(self):
        if self.dynamic_density:
            #Define STD SL Terms
            ##P_0 = 101325 #Pa
            L = 0.0065 #K/m
            M = 0.0289644 #kg/mol
            R = 8.31447 #J/(mol K)
            g = 9.80665 #m/s^2
            T_0 = 288.15 #K
            p = P_0*(1 + (L*0.3048*self.state0[2])/T_0)**(g*M/(R*L))
            return p*M/(R*T) * (0.00194) #Covert to slug/ft^3
        else:
            return 0.002377 #SL slug/ft^3

    def update_state0(self, new_state):
        if len(new_state) != 13:
            print("Error - Not 13 items! \n You might need to convert angular\
                    values to quaternions...")
        else:
            self.state0 = new_state

    def CL_stall(self, alpha, model, coeff):
        from math import exp, sin, cos
        from numpy import sign
        [discard, M, alpha_star] = model
        del_alpha = alpha - alpha_star
        add_alpha = alpha + alpha_star
        sig = (1+exp(-M*del_alpha)+exp(M*add_alpha))/((1+exp(-M*del_alpha))*(1+exp(M*add_alpha)))
        CL = (1-sig)*(coeff[0]+coeff[1]*alpha) + sig*2*sign(alpha)*(sin(alpha)**2)*cos(alpha)
        return CL

    def aero_terms(self):
        from integrator import EP2Euler321
        from math import atan
        from numpy import matmul, transpose

        [u, v, w] = self.state0[3:6]
        V_t = (u**2 + v**2 + w**2)**(1/2)  #NOTE: No wind included
        Q = 0.5 * V_t**2 *self.density()

        alpha = atan(-w/u)  #NOTE: Negative sign since Pd is inverted
        beta = atan(v/u)
        angles = [alpha, beta]

        #Determine if stall model needed
        wing_stall = self.wing[5][0]
        h_stall = self.hstab[5][0]
        v_stall = self.vstab[5][0]

        #Forces are [-D S -L] in wind frame
        coeff = self.wing[6]
        if wing_stall:
            wing_w = [-Q*self.wing[0]*self.wing[1]*(coeff[4] + coeff[5]*alpha + coeff[6]*self.wing[1]/(2*V_t)*self.state0[11]),
                    0,
                    -Q*self.wing[0]*self.wing[1]*(self.CL_stall(alpha, self.wing[5], coeff[0:2]) + coeff[2]*self.wing[1]/(2*V_t)*self.state0[11])]
        else:
            wing_w = [-Q*self.wing[0]*self.wing[1]*(coeff[4] + coeff[5]*alpha + coeff[6]*self.wing[1]/(2*V_t)*self.state0[11]),
                    0,
                    -Q*self.wing[0]*self.wing[1]*(coeff[0] + coeff[1]*alpha + coeff[2]*self.wing[1]/(2*V_t)*self.state0[11])]

        coeff = self.hstab[6]
        if h_stall:
            hstab_w = [-Q*self.hstab[0]*self.hstab[1]*(coeff[4] + coeff[5]*alpha + coeff[7]*self.controls[0]),
                        0,
                        -Q*self.hstab[0]*self.hstab[1]*(self.CL_stall(alpha, self.hstab[5], coeff[0:2]) + coeff[3]*self.controls[0])]
        else:
            hstab_w = [-Q*self.hstab[0]*self.hstab[1]*(coeff[4] + coeff[5]*alpha + coeff[7]*self.controls[0]),
                        0,
                        -Q*self.hstab[0]*self.hstab[1]*(coeff[0] + coeff[1]*alpha + coeff[3]*self.controls[0])]

        coeff = self.vstab[6]
        if v_stall:
            vstab_w = [-Q*self.vstab[0]*self.vstab[1]*(coeff[5] * beta + coeff[9]*self.controls[3]),
                        Q*self.vstab[0]*self.vstab[1]*(self.CL_stall(beta, self.vstab[5], coeff[0:2]) + coeff[3]*self.controls[3]),
                        0]
        else:
            vstab_w = [-Q*self.vstab[0]*self.vstab[1]*(coeff[4] + coeff[5]*beta + coeff[7]*self.controls[3]),
                        Q*self.vstab[0]*self.vstab[1]*(coeff[0] + coeff[1]*beta + coeff[3]*self.controls[3]),
                        0]

        #Forces in BODY frame
        XYZ_w = list(map(sum, zip( list(map(sum, zip(wing_w, hstab_w))) , vstab_w)))
        [X, Y, Z] = w2b(angles, XYZ_w)

        #Condense moment inputs
        #ROLL
        w_roll_w = [-Q*self.wing[0]*self.wing[1]*self.wing[3]*self.wing[6][7]*2*self.controls[2],
                        0,
                        -Q*self.wing[0]*self.wing[1]*self.wing[3]*self.wing[6][3]*2*self.controls[2]]
        w_roll_b = matmul([0, 0, self.wing[3]], w2b(angles, w_roll_w))
        #w_roll_b +=  Q*self.wing[0]*(self.wing[1]**2)*self.wing[6][8][0]*self.controls[2] #Roll moment of aileron
        v_roll_b = matmul([0, self.vstab[4], 0], w2b(angles, vstab_w))
        #v_roll_b +=  Q*self.vstab[0]*(self.vstab[1]**2)*self.vstab[6][8][0]*self.controls[3] #Roll moment of rudder
        L = w_roll_b+v_roll_b

        #PITCH
        h_pitch_b = matmul([0, 0, self.hstab[2]], w2b(angles, hstab_w))
        #h_roll_b +=  Q*self.hstab[0]*(self.hstab[1]**2)*self.hstab[6][8][1]*self.controls[0] #Pitch moment of elevator
        w_pitch_b = matmul([0, 0, self.wing[2]], w2b(angles, wing_w))
        M = h_pitch_b+w_pitch_b

        #YAW
        v_yaw_b = matmul([0, self.vstab[2], 0], w2b(angles, vstab_w))
        #w_roll_b =  Q*self.wing[0]*(self.wing[1]**2)*self.wing[6][8][2]*self.controls[2] #Pitch moment of aileron
        N = v_yaw_b #+w_roll_b

        #DEBUG AREA
        print('AERO TERMS:')
        print([X, Y, Z, L, M, N])
        from time import sleep
        sleep(1)
        return [X, Y, Z, L, M, N]

    def update_FM(self, t):
        from math import sin, cos
        from integrator import EP2Euler321

        #Angularize Gravity
        angles = EP2Euler321(self.state0[6:10])
        Fg = f2b(angles, [0, 0, 32.2*self.mass])

        #All Forcing Functions
        for i in range(6):
            try:
                self.FM[i] = self.FMeq[i](t)
            except:
                self.FM[i] = self.FMeq[i]

        #Add in Gravity
        self.FM[0] += Fg[0] + self.thrust_max * self.controls[1] #Add in thrust
        self.FM[1] += Fg[1]
        self.FM[2] += Fg[2]

        #Add in Aero Terms
        aero_b = self.aero_terms()
        self.FM[0] += aero_b[0]
        self.FM[1] += aero_b[1]
        self.FM[2] += aero_b[2]
        self.FM[0] += aero_b[3]
        self.FM[1] += aero_b[4]
        self.FM[2] += aero_b[5]

        return self.FM

    #===============================================================
    #Add templated aircraft below this line to pre-generate aircraft
    #===============================================================
    def hw1_1(self):
        import warnings
        warnings.warn("This aircraft is depreciated", DeprecationWarning)
        self.state0 = [100, 200, -500, 50, 0, 0,
                        0.70643, 0.03084, 0.21263, 0.67438, 0, 0, 0]
        self.FMeq = [0, 0, 0, 0, 0, 0]

    def hw1_2(self):
        import warnings
        warnings.warn("This aircraft is depreciated", DeprecationWarning)
        from math import sin, cos
        self.state0 = [100, 200, -500, 50, 0, 0,
                        0.70643, 0.03084, 0.21263, 0.67438, 0, 0, 0]
        self.FMeq = [(lambda t: sin(t)), 0, 0,
                        0, 1e-4, 0]

    def hw2(self):
        #All units listed in English units as denoted
        #NOTE: alpha and beta in radians
        self.mass = 0.925  # Mass (slug)
        self.dynamic_density = False #True uses lapse rate for rho=f(h)

        #Inert = [Ixz, Ix, Iy, Iz]
        self.inert = [2.857, 19.55, 26.934, 14.74]  # Moment of Inertia (lbf*ft^2)

        #State = [p_n, p_e, p_d, u, v, w, e0, e1, e2, e3, p, q, r]
        self.state0 = [0, 0, 0, 20, 0, 0, 1, 0, 0, 0, 0, 0, 0]
             #Level flight at 0ft at 20 ft/s

        #FM = [Fx, Fy, Fz, Ell, M, N]
        self.FM = [0, 0, 0, 0, 0, 0]
             #Equations for time-variant forces and moments
        self.FMeq = [0, 0, 0, 0, 0, 0]

        self.thrust_max = 5.62  #lbf

        #Geometric Properties and Coefficients
         #Order: b, c, x_cg, y_cg, z_cg, stall, coefficients
             #stall: True/False, M, alpha_0
             #coeff: CL0, CLa/b, CLq, CL_del_control, CD0, CDa/b, CDq, CD_del_control, [spare]
        self.wing = [9.413, 0.6791, 0.9843, 0, 0, [True, 50, 0.471],
                     [0.28, 3.5, 0, 0, 0.015, 0, 0, 0, [0.08, 0, 0.06]]]
        self.hstab = [2.297, 0.381, 0.8202, 0, 0, [False, 0, 0],
                     [0.1, 5.79, 0, 0.36, 0.01, 0, 0, 0, [0, -0.5, 0]]]
        self.vstab = [.9843, 0.381, 0.8202, 0, 0, [False, 0, 0],
                     [0, 5.79, 0, -0.17, 0.01, 0, 0, 0, [0.105, 0, 0]]]

        #Controls Deflections: del_e, del_t(<1), del_a, del_r
        self.controls = [ 0, 1, 0, 0]
