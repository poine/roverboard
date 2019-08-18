#!/usr/bin/env python
import sys, numpy as np
import rospy, geometry_msgs.msg

import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.trr.utils as trr_u
import calibrate_odom as clbod
import pdb

class TurnInPlaceCalib:
    def __init__(self, d=np.deg2rad(90)):
        self.name = 'turn_in_place'
        self.d = d
        self.psi0 = None
    def started(self): return self.psi0 is not None
    def start(self, p0, psi0):
        self.psi0 = psi0
    def run(self, p, psi, om=0.5):
        dpsi = psi - self.psi0; self.dist = trr_u.norm_mpi_pi(dpsi)
        return (0, om, self.dist)
    def finished(self): return self.dist > self.d
    
def main(args): clbod.Node(TurnInPlaceCalib()).run(50.)

if __name__ == '__main__':
    main(sys.argv)
