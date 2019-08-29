#!/usr/bin/env python
import time, math
import numpy as np


import christine_hwi_ext

import pdb

#
# Drive circle of given radius
#
#   can be used to check steering servo
#
class Controller:
    
    def __init__(self, vel_cps, R):
        self.vel_cps = vel_cps
        self.R = R      # Radius of circle
        self.L = 0.27   # Distance between front and rear wheels

    def get(self): return self.vel_cps, np.arctan2(self.L, self.R)

    def alpha_to_servo(self, _a): return 1.508*_a
        
    # 0.75 -> 1.520
    # 1.25 -> 1.535
    # 2.   -> 1.523

    
def drive(bbbl, vel_cps, R, d_psi=np.pi):
    ctl = Controller(vel_cps, R)
    while True:
        vel_cps, alpha = ctl.get()
        srv = ctl.alpha_to_servo(alpha)
        print('alpha {:.2f} deg servo {:.1f} %'.format(np.rad2deg(alpha), srv*100.))
        bbbl.send(srv, vel_cps)
        time.sleep(0.02)

def main():
    bbbl = christine_hwi_ext.BBBLink()
    bbbl.init()
    drive(bbbl, vel_cps=60, R=0.75)


if __name__ == '__main__':
    main()
