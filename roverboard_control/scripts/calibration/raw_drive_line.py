#!/usr/bin/env python
import time, math
import numpy as np


import christine_hwi_ext

import pdb

#
# Drive line
#
#   can be used to check wheel radius
#
class Controller:
    
    def __init__(self, vel_cps):
        self.vel_cps = vel_cps
        self.enc_cpr = 64.5/(2*np.pi)
        self.wr = 0.041

    def reset(self, _ec0):
        print('reset at {}'.format(_ec0))
        self.ec0 = _ec0
        
    def get(self): return self.vel_cps

    def encs_to_dist(self, _e): return self.encc_to_rad(_e)*self.wr
    def encc_to_rad(self, _e): return _e/self.enc_cpr

    def get_dist(self, enc_pos): return self.encs_to_dist(enc_pos-self.ec0)
    
def drive(bbbl, vel_cps, _dist=1.):
    ctl = Controller(vel_cps)
    time.sleep(0.1)
    ctl.reset(bbbl.get_motor()[0])
    while True:
        dist = ctl.get_dist(bbbl.get_motor()[0])
        dist
        if dist > _dist: break
        vel_cps = ctl.get()
        bbbl.send(0, vel_cps)
        time.sleep(0.02)

    bbbl.send(0, 0)
    time.sleep(0.1)
    dist = ctl.get_dist(bbbl.get_motor()[0])
    print('dist: {:.2f} m'.format(dist))
    
def main():
    bbbl = christine_hwi_ext.BBBLink()
    bbbl.init()
    drive(bbbl, vel_cps=60, _dist=5.)


if __name__ == '__main__':
    main()
