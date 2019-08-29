#!/usr/bin/env python
import time, math
import numpy as np
import scipy.signal

import odrive_ascii_cpp_ext
import christine_hwi_ext

import two_d_guidance.utils as tdgu
import pdb

#
# Spin the wheels by a given amount of revolutions
#
#   can be used to check reduction and encoder resolution
#

class Controller:
    
    def __init__(self, omega=4, nb_rev=5, vel_cpr=60.):
        self.nb_rev, self.vel_cpr = nb_rev, vel_cpr
        self.enc_cpr = 64.5/(2*np.pi)
        self.m1_ref = tdgu.SecOrdLinRef(omega=omega, xi=0.9)

    def init(self, m1_pos_meas0):
        print('initialized at {}'.format(m1_pos_meas0))
        self.m1_pos_meas0 = m1_pos_meas0
        
    def get(self, m1sp, dt, m1_pos_meas, deadband=0.):
        d_cpr = m1_pos_meas - self.m1_pos_meas0
        sp = self.vel_cpr if self.encc_to_rad(d_cpr) < self.nb_rev*2*np.pi else 0
        m1_refX = self.m1_ref.run(dt, sp)
        return m1_refX if abs(m1_refX[0])>=deadband else 0.

    def encc_to_rad(self, _e): return _e/self.enc_cpr
    def rad_to_encc(self, _r): return _r*self.enc_cpr
    

def measure(od, bbbl, dt=0.02, nb_rev=10., vel_cpr=60, filename='/tmp/odrive_vel_step_lifted2.npz'):
    ctl = Controller(nb_rev=nb_rev, vel_cpr=vel_cpr)
    _nb_rad = nb_rev*2*np.pi
    _nb_encc = ctl.rad_to_encc(_nb_rad)
    dur = _nb_encc/vel_cpr + 1.
    print('nb_rev {:.1f} vel {:.1f} cps duration {:.1f} s'.format(nb_rev, vel_cpr, dur))
    _time = np.arange(0, dur, dt)
    _time_r = np.zeros(len(_time))
    wheel_sp_cpr = np.zeros((len(_time), 1))     # in cpr/s
    wheel_ref_cpr = np.zeros((len(_time), 2, 3)) #
    mot_i_ff = np.zeros((len(_time),2))          #
    enc_pos, enc_vel = np.zeros((len(_time),2)), np.zeros((len(_time),2))
    if od is not None:
        enc_pos[-1,0], enc_pos[-1,1], enc_vel[-1, 0], enc_vel[-1,1] = od.read_feedback()
    else:
        time.sleep(0.1)
        enc_pos[-1,0], enc_vel[-1, 0] = bbbl.get_motor()

    ctl.init(enc_pos[-1,0])
    for i in range(len(_time)):
        _time_r[i] = time.time()
        wheel_ref_cpr[i,0] = ctl.get(wheel_sp_cpr[i,0], dt, enc_pos[i-1,0])
        if od is not None:
            od.send_velocity_setpoint(wheel_ref_cpr[i,0, 0], 0, 0, 0)
            enc_pos[i,0], enc_pos[i,1], enc_vel[i, 0], enc_vel[i,1] = od.read_feedback()
        else:
            bbbl.send(0, wheel_ref_cpr[i,0, 0])
            enc_pos[i,0], enc_vel[i, 0] = bbbl.get_motor()
            
        #if i%10 == 0: print('enc pos {}'.format(enc_pos[i,0]))
        _end = time.time()
        next_loop_time = _time_r[0] + dt*(i+1)
        sleep_time = next_loop_time - _end
        #print('start:{} end:{} next:{} sleep:{}'.format(_time_r[i], _end, next_loop_time, sleep_time))
        if sleep_time >0:
            time.sleep(sleep_time)
    if od is not None:
        od.send_velocity_setpoint(0, 0, 0, 0)
    else:
        bbbl.send(0, 0)
    d_cpr = enc_pos[-1,0] - ctl.m1_pos_meas0
    d_rad = ctl.encc_to_rad(d_cpr)
    d_rev = d_rad/(2*np.pi)
    print('result: cprs {:.1f} rad {:.3f} rev {:.2f}'.format(d_cpr, d_rad, d_rev))
    
    
    mot_a_sp, mot_a_meas = np.zeros((len(_time),2)), np.zeros((len(_time),2))
    np.savez(filename, _time=_time, _time_r=_time_r, wheel_sp_cpr=wheel_sp_cpr, enc_pos=enc_pos, enc_vel=enc_vel,
             mot_a_sp=mot_a_sp, mot_a_meas=mot_a_meas, wheel_ref_cpr=wheel_ref_cpr)



    
def main():
    use_bbbl = True

    if not use_bbbl:
        od = odrive_ascii_cpp_ext.Odrive()
        od.init()
        bbbl = None
    else:
        bbbl = christine_hwi_ext.BBBLink()
        bbbl.init()
        od = None

    measure(od, bbbl, nb_rev=1.)
    
if __name__ == '__main__':
    main()
