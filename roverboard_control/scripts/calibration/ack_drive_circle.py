#!/usr/bin/env python
import sys, time, numpy as np
import rospy, ackermann_msgs.msg

''' 
Drive a circle.
 still needs some love
'''

import two_d_guidance.utils as tdgu
import two_d_guidance.trr.rospy_utils as trr_rpu
import ack_node

class Ctl:
    def __init__(self, _v, _R):
        self.v, self.R = _v, _R;
        self.d = 1.
        self.L = 0.27   # Distance between wheels
        self.alpha = np.arctan2(self.L, self.R)
        self.p0=None
    def initialize(self, p0, psi0):
        self.p0 = p0
        self.psi0 = psi0
        print psi0
    def initialized(self): return self.p0 is not None
    def finished(self, p, psi):
        d_psi = psi-self.psi0
        return tdgu.normalize_headings(d_psi) > np.pi/2
    def get(self, p, psi):
        return (self.alpha, self.v) if not self.finished(p, psi) else (0., 0.)
    def _dist(self, p): return np.linalg.norm(p-self.p0)
    def _ang(self, psi): return psi-self.psi0
            
def main(args):
    ack_node.Node(Ctl(_v=0.2, _R=1.25)).run(50.)

if __name__ == '__main__':
    main(sys.argv)
