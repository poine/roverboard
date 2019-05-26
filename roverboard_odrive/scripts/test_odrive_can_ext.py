#!/usr/bin/env python
import time
import math, numpy as np

import odrive_can_cpp_ext

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def stairs(t,  n_stair=10, dt_stair=0.5, _min=0, _max=20):
    a = int(math.fmod(t, dt_stair*n_stair)/dt_stair)
    return _min + (_max - _min) * a / n_stair

class SineSetpoint:
    
    def get(self, t):
        return 90.*np.sin([t, t])

class StepSetpoint:
    def __init__(self, a0=-50., a1=50., dt=2., t0=0):
        self.a0, self.a1, self.dt, self.t0 = a0, a1, dt, t0
        
    def get(self, t):
        v = step(t, self.a0, self.a1, self.dt, self.t0)
        return [v, v]

class StairSetpoint:
    def __init__(self, a0=-75., a1=75, stairs=15, dt=2., t0=0):
        self.a0, self.a1, self.dt, self.stairs, self.t0 = a0, a1, dt, stairs, t0
        
    def get(self, t):
        v = stairs(t, self.stairs, self.dt, self.a0, self.a1)
        return [v, v]
    
def measure(od, dt=0.01, dur=40., filename='/tmp/odrive_vel_step_lifted2.npz'):
    #sp = SineSetpoint()
    sp = StepSetpoint()
    #sp = StairSetpoint()
    _time = np.arange(0, dur, dt)
    _time_r = np.zeros(len(_time))
    wheel_sp_cpr = np.zeros((len(_time), 2)) # in cpr/s
    enc_pos, enc_vel = np.zeros((len(_time),2)), np.zeros((len(_time),2))
    mot_a_sp, mot_a_meas = np.zeros((len(_time),2)), np.zeros((len(_time),2))
    for i in range(len(_time)):
        _time_r[i] = time.time()
        wheel_sp_cpr[i] = sp.get(_time_r[i])
        od.send_velocity_setpoint(wheel_sp_cpr[i,0], wheel_sp_cpr[i,1])
        enc_pos[i,0], enc_pos[i,1], enc_vel[i,0], enc_vel[i,1], mot_a_sp[i,0], mot_a_sp[i,1], mot_a_meas[i,0], mot_a_meas[i,1] = od.read_feedback()
        _end = time.time()
        #print(mot_a_sp[i,0], mot_a_sp[i,1], mot_a_meas[i,0], mot_a_meas[i,1])
        next_loop_time = _time_r[0] + dt*(i+1)
        sleep_time = next_loop_time - _end
        #print('start:{} end:{} next:{} sleep:{}'.format(_time_r[i], _end, next_loop_time, sleep_time))
        if sleep_time >0:
            time.sleep(sleep_time)
    od.send_velocity_setpoint(0, 0)
    
    
    np.savez(filename, _time=_time, _time_r=_time_r, wheel_sp_cpr=wheel_sp_cpr, enc_pos=enc_pos, enc_vel=enc_vel,
             mot_a_sp=mot_a_sp, mot_a_meas=mot_a_meas)

def test(od):
    od.send_velocity_setpoint(50, 50)
    print( od.read_feedback())
    time.sleep(0.5)
    print( od.read_feedback())
    od.send_velocity_setpoint(0, 0)

def main():
    od = odrive_can_cpp_ext.Odrive()
    od.init()
    #test(od)
    measure(od)

if __name__ == '__main__':
    main()
