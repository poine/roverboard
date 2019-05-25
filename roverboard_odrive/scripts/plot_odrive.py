#!/usr/bin/env python
import time
import numpy as np, matplotlib.pyplot as plt

def main(filename="/tmp/odrive_vel_step_lifted2.npz"):
   data = np.load(filename)
   _time, time_r = data['_time'], data['time_r']
   wheel_sp_cpr, enc_pos = data['wheel_sp_cpr'], self.data['enc_pos']
   plt.plot(_time, enc_pos)
   plt.show()

if __name__ == '__main__':
    main()
