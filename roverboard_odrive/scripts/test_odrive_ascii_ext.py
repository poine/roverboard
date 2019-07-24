#!/usr/bin/env python
import time
import numpy as np

import odrive_ascii_cpp_ext

def measure(od, dt=0.02, dur=10., filename='/tmp/odrive_vel_step_lifted2.npz'):
    _time = np.arange(0, dur, dt)
    _time_r = np.zeros(len(_time))
    wheel_sp_cpr = np.zeros((len(_time), 2)) # in cpr/s
    wheel_sp_cpr[:,0] = 90*np.sin(_time)
    wheel_sp_cpr[:,1] = 90*np.sin(_time)
    enc_pos, enc_vel = np.zeros((len(_time),2)), np.zeros((len(_time),2))
    for i in range(len(_time)):
        _time_r[i] = time.time()
        od.send_velocity_setpoint(wheel_sp_cpr[i,0], wheel_sp_cpr[i,1], 0, 0)
        enc_pos[i,0], enc_pos[i,1], enc_vel[i,0], enc_vel[i,1] = od.read_feedback()
        _end = time.time()
        next_loop_time = _time_r[0] + dt*(i+1)
        sleep_time = next_loop_time - _end
        #print('start:{} end:{} next:{} sleep:{}'.format(_time_r[i], _end, next_loop_time, sleep_time))
        if sleep_time >0:
            time.sleep(sleep_time)
    od.send_velocity_setpoint(0, 0)
    
    
    mot_a_sp, mot_a_meas = np.zeros((len(_time),2)), np.zeros((len(_time),2))
    np.savez(filename, _time=_time, _time_r=_time_r, wheel_sp_cpr=wheel_sp_cpr, enc_pos=enc_pos, enc_vel=enc_vel,
             mot_a_sp=mot_a_sp, mot_a_meas=mot_a_meas)
          
def test():
    od.send_velocity_setpoint(50, 50, 0, 0)
    print(od.read_feedback())
    time.sleep(0.5)
    print(od.read_feedback())
    od.send_velocity_setpoint(0, 0)
        
od = odrive_ascii_cpp_ext.Odrive()
od.init()
time.sleep(0.1)
measure(od)
