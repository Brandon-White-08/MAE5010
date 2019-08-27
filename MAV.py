#					TITLE BLOCK
#**************************************************
#Author:	Brandon White
#Date:		08/26/2019
#Desc:		Creates a MAV object with mass, moment
#           of inertia, and gravity properties
#**************************************************

#Calling the class with an aircraft name below creates an MAV object
class MAV:
    def __init__(self, aircraft = "None"):
       #All masses listed in English units as denoted
       self.name = aircraft
       self.mass = 100  # Mass (lbf)
       self.inert = [10, 10, 10, 10]  # Moment of Inertia (lbf*ft^2)
       self.gravity_needed = False

       if aircraft != "None":
           try:
               method_to_call = getattr(self, aircraft.lower())
               method_to_call()
           except:
               print("No preconfig by given name: " + aircraft.lower())

    def update_mass(self, new_mass):
        self.mass = new_mass

    def update_inert(self, new_inert):
        self.inert = new_inert

    def toggle_gravity(self):
        if self.gravity_needed:
            self.gravity_needed = False
        else:
            MAV.gravity_needed = True
        print("Gravity Needed toggled to: " + str(MAV.gravity_needed))

    #Add templated aircraft below this line to pregenerate aircraft
    def temp(self):
        self.mass = 500
        self.update_inert([200, 50, 50, 50])
