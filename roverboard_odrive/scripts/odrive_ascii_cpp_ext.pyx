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
    cdef cppclass c_Odrive "Odrive":
        c_Odrive()
        bool init()
        void send_velocity_setpoint(int m1, int m2)
        void read_feedback(double* enc, double* enc_vel)

cdef class Odrive:
    cdef c_Odrive *thisptr

    def __cinit__(self):
        self.thisptr = new c_Odrive()

    def init(self):
        self.thisptr.init()
        
    def send_velocity_setpoint(self, m0, m1):
        self.thisptr.send_velocity_setpoint(m0, m1)

    def read_feedback(self):
        cdef double enc[2]
        cdef double enc_vel[2]
        self.thisptr.read_feedback(enc, enc_vel)
        return enc[0], enc[1], enc_vel[0], enc_vel[1]
