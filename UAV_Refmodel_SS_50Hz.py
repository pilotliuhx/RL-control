import numpy as np

class UAV_SS_Refmodel_Class():

    def __init__(self, roll0):
        
        # initial states
        self.refroll = roll0
        self.refrollrate = 0
        self.dt = 0.02
        
        # state space 
        self.x1 = 0.0
        self.x2 = self.refroll
        # G matrix
        self.G1, self.G2= 0.863, -0.3135
        self.G3, self.G4= 0.0186,  0.9968
        # H vector
        self.H1, self.H2= 0.3135, 0.00321
        # C vector
        self.C1, self.C2 = 0, 1
        
        
    def reset(self, roll0):
        # reset all states
        self.x1 = 0.0
        self.x2 = roll0
        self.roll = roll0
        self.rollrate = 0.0
        
    def step(self, u):
        
        self.x1 = self.G1*self.x1 + self.G2*self.x2 + self.H1*u
        self.x2 = self.G3*self.x1 + self.G4*self.x2 + self.H2*u
        
        self.refrollrate = self.x1
        self.refroll     = self.x2
        
        return self.refroll, self.refrollrate
        
        
        
        
        
        
        
        
    