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
       self.name = aircraft
       self.mass = 10  # Mass (lbf)
       #Inert = [Ixz, Ix, Iy, Iz]
       self.inert = [20, 10, 10, 10]  # Moment of Inertia (lbf*ft^2)
       self.gravity_needed = False
       #State = [p_n, p_e, p_d, u, v, w, e0, e1, e2, e3, p, q, r]
       self.state0 = [0, 0, -500, 50, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            #Level flight at 500 ft at 50 ft/s
       #FM = [Fx, Fy, Fz, Ell, M, N]
       self.FM = [0, 0, 0, 0, 0, 0]
            #Gravity ONLY in base model
       self.FMeq = [0, 0, (lambda t: 32.2*self.mass), 0, 0, 0]
       #Stability Derivatives/Coefficients
        #Order:
       self.wingCoeff = [0, 0, 0, 0, 0, 0, 0, 0]
       self.hstabCoeff = [0, 0, 0, 0, 0, 0, 0, 0]
       self.vstabCoeff = [0, 0, 0, 0, 0, 0, 0, 0]
       self.coeffeq = [0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0]

       if aircraft != "None":
           try:
               method_to_call = getattr(self, aircraft.lower())
               method_to_call()
           except:
               print("No preconfig by given name: " + aircraft.lower())

    def update_state0(self, new_state):
        if len(new_state) != 13:
            print("Error - Not 13 items! \n You might need to convert angular\
                    values to quaternions...")
        else:
            self.state0 = new_state

    def update_FM(self, t):
        from math import sin, cos
        from integrator import EP2Euler321

        #Angularize Gravity
        angles = EP2Euler321(self.state0[6:10])
        Fg = f2b(angles, [0, 0, 32.2*self.mass])

        #All Other Forcing Functions
        for i in range(6):
            try:
                self.FM[i] = self.FMeq[i](t)
            except:
                self.FM[i] = self.FMeq[i]

        #Add in Gravity
        self.FM[0] += Fg[0]
        self.FM[1] += Fg[1]
        self.FM[2] += Fg[2]

        return self.FM

    #Add templated aircraft below this line to pregenerate aircraft
    def hw1_1(self):
        self.state0 = [100, 200, -500, 50, 0, 0,
                        0.70643, 0.03084, 0.21263, 0.67438, 0, 0, 0]
        self.FMeq = [0, 0, 0, 0, 0, 0]

    def hw1_2(self):
        from math import sin, cos
        self.state0 = [100, 200, -500, 50, 0, 0,
                        0.70643, 0.03084, 0.21263, 0.67438, 0, 0, 0]
        self.FMeq = [(lambda t: sin(t)), 0, 0,
                        0, 1e-4, 0]

    def hw2_737max(self):
        
