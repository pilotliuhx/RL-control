import numpy as np

class UAV_SSmodel_Class():

    def __init__(self, roll0):
        
        # initial states
        self.roll = roll0
        self.rollrate = 0
        self.dt = 0.02
        
        # state space 
        self.x1 = 0.0
        self.x2 = self.roll
        # G matrix
        self.G1, self.G2= 0.7627, 0
        self.G3, self.G4= 0.0175,  1
        # H vector
        self.H1, self.H2= 0.2373, 0.00248
        # C vector
        self.C1, self.C2 = 1, 0
        
        
    def reset(self, roll0):
        # reset all states
        self.x1 = 0.0
        self.x2 = roll0
        self.roll = roll0
        self.rollrate = 0.0
        
    def step(self, u):
        
        self.x1 = self.G1*self.x1 + self.G2*self.x2 + self.H1*u
        self.x2 = self.G3*self.x1 + self.G4*self.x2 + self.H2*u
        
        self.rollrate = self.x1
        self.roll     = self.x2
        
        return self.roll, self.rollrate
        
        
        
        
        
        
        
        
    