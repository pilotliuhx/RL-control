'''

UAV Simulator roll angle

Attitude Model with State Space

Jan. 22 2021

Yuanda Wang

'''

import numpy as np
from UAV_model_SS_50Hz import UAV_SSmodel_Class 
from UAV_Refmodel_SS_50Hz import UAV_SS_Refmodel_Class
def generate_init_atti(max_angle):
    co = 0.99
    roll   = (2 * np.random.random() - 1) * max_angle * co 
    
    return roll

def generate_d_atti(max_angle):
    co = 0.99
    roll   = (2 * np.random.random() - 1) * max_angle * co

    return roll


class UAV_Attitude_Simulator():
    def __init__(self):
        self.dt = 0.02
        self.max_rate = 2.5
        self.max_atti = np.pi / 4
        self.done = False
        self.i = 0.0
        roll0  = generate_init_atti(self.max_atti)
        # initialize UAV SS model
        self.uav = UAV_SSmodel_Class(roll0)
        self.refmodel = UAV_SS_Refmodel_Class(roll0)
        # set desired attitude
        self.d_atti = generate_d_atti(self.max_atti)
        
    
    def reset(self):
        self.done = False
        # reset init roll angle
        atti0  = generate_init_atti(self.max_atti)
        self.uav.reset(atti0)
        self.refmodel.reset(atti0)
        # reset desired attitude
        self.d_atti = generate_d_atti(self.max_atti)
        # observe current state
        ob = self.observe()
        
        return ob
        
    def check_fail(self):
        CHECK_ATTI = True
        CHECK_RATE = False
        
        self.ATTI_FAIL = False
        self.RATE_FAIL = False
        
        # check atti angle out of range
        roll = self.uav.roll
        if CHECK_ATTI and (np.absolute(roll) > self.max_atti):
            self.ATTI_FAIL = True
            self.done = True
            
    def reward(self):
        e_roll = abs(self.refmodel.refroll - self.uav.roll) + abs(self.refmodel.refrollrate - self.uav.rollrate) + abs(self.i)
        self.r = - abs(e_roll)
        
        return self.r
        
        
    def observe(self):
        rollrate  = self.uav.rollrate
        roll   = self.uav.roll
        d_roll  = self.d_atti
        ref_roll = self.refmodel.refroll
        ref_rollrate = self.refmodel.refrollrate
        # no need to normalize
        
        # noise
        # rollrate += np.random.normal(0.0, 0.1)
        # pitchrate += np.random.normal(0.0, 0.1)
        
        # calc attitude error
        e_roll  = ref_roll - roll
        e_rollrate = ref_rollrate - rollrate
        self.atti_errors = e_roll  
        self.i = self.i + e_roll * self.dt
        ob = np.array([e_roll, e_rollrate, self.i])
        
        return ob
        
        
    
    def step(self, u):
        
        # update model
        self.uav.step(u)
        self.refmodel.step(self.d_atti)
        # make observation
        ob = self.observe()
        
        # give reward
        r = self.reward()
        
        self.check_fail()
        
        
        # if atti out of range
        done = self.done
        
        return ob, r, done
    
    
    
    
        
