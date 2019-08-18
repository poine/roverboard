#!/usr/bin/env python
import sys, numpy as np
import rospy, geometry_msgs.msg

import two_d_guidance.trr.rospy_utils as trr_rpu
import calibrate_odom as clbod
import pdb

class StraightLineCalib:
    def __init__(self, d=1.):
        self.name = 'one_meter_line'
        self.d = d
        self.p0 = None
    def started(self): return self.p0 is not None
    def start(self, p0, psi0):
        self.p0 = p0
    def run(self, p, psi, v=0.1):
        dep = p - self.p0; self.dist = np.linalg.norm(dep)
        return (v, 0., self.dist)
    def finished(self): return self.dist > self.d

def main(args): clbod.Node(StraightLineCalib()).run(50.)

if __name__ == '__main__':
    main(sys.argv)
