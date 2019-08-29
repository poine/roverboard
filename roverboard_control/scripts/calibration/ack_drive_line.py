#!/usr/bin/env python
import sys, time, numpy as np
import rospy, ackermann_msgs.msg

''' 
Drive a line.
 still needs some love
'''

import two_d_guidance.trr.rospy_utils as trr_rpu
import ack_node

class Ctl:
    def __init__(self, _v, _d): self.v, self.d = _v, _d; self.p0=None
    def initialize(self, p0, psi0): self.p0 = p0
    def initialized(self): return self.p0 is not None
    def finished(self, p, psi): return self._dist(p) > self.d
    def get(self, p, psi): return (0., self.v) if not self.finished(p, psi) else (0., 0.)
    def _dist(self, p): return np.linalg.norm(p-self.p0)
            
def main(args):
    ack_node.Node(Ctl(_v=0.2, _d=1.)).run(50.)

if __name__ == '__main__':
    main(sys.argv)
