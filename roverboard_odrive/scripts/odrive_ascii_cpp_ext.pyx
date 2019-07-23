# distutils: language = c++
#
#
#  cython interface to the C++ ascii odrive code 
#
# Author: Poine-2019
#

# I should look at that https://github.com/longjie/ros_cython_example

import numpy as np

from libcpp cimport bool

cdef extern from "roverboard_odrive/roverboard_odrive_ascii.h":
    cdef cppclass c_Odrive "OdriveAscii":
        c_Odrive()
        bool init()
        void readFeedback(double* enc, double* enc_vel, double* iq_sp, double* iq_meas)
        void sendVelSetpoints(double* vsps, double* iq_ff)


cdef class Odrive:
    cdef c_Odrive *thisptr

    def __cinit__(self):
        self.thisptr = new c_Odrive()

    def init(self):
        self.thisptr.init()
        
    def send_velocity_setpoint(self, m0, m1):
        cdef double vsp[2]
        cdef double iff[2]
        vsp[0] = m0; vsp[1] = m1
        iff[0] = 0; iff[1] = 0
        self.thisptr.sendVelSetpoints(vsp, iff)

    def read_feedback(self):
        cdef double enc[2]
        cdef double enc_vel[2]
        cdef double iq_sp[2]
        cdef double iq_meas[2]
        self.thisptr.readFeedback(enc, enc_vel, iq_sp, iq_meas)
        return enc[0], enc[1], enc_vel[0], enc_vel[1]
