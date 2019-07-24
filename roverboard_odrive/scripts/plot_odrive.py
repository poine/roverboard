#!/usr/bin/env python
import time
import numpy as np, matplotlib.pyplot as plt
from scipy import signal

import two_d_guidance.plot_utils as tpu

class Dataset:
   def __init__(self, filename):
      self.data = np.load(filename)
      self._time, self._time_r = self.data['_time'], self.data['_time_r']
      self.wheel_sp_cpr, self.enc_pos, self.enc_vel = self.data['wheel_sp_cpr'], self.data['enc_pos'], self.data['enc_vel']
      self.mot_a_sp, self.mot_a_meas = self.data['mot_a_sp'], self.data['mot_a_meas']
      

   def plot_chronogram(self):
      tpu.prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None)
      s = slice(1, len(self._time_r), 1)
      b, a = signal.butter(4, 0.03, analog=False)

      ax = plt.subplot(3,2,1)
      plt.plot(self._time_r[s], self.enc_pos[s,0])
      tpu.decorate(ax, title="chan0 pos", xlab='ticks', ylab='time', legend='encoders')
      ax = plt.subplot(3,2,2)
      plt.plot(self._time_r[s], self.enc_pos[s,1])
      tpu.decorate(ax, title="chan1 pos", xlab='ticks', ylab='time', legend='encoders')
      ax = plt.subplot(3,2,3)
      plt.plot(self._time_r[s], self.wheel_sp_cpr[s,0], label='sp')
      plt.plot(self._time_r[s], self.enc_vel[s,0], label='meas', alpha=0.5)
      mot0_enc_vel_flt = signal.filtfilt(b, a,  self.enc_vel[s,0])
      plt.plot(self._time_r[s], mot0_enc_vel_flt, label='flt')
      tpu.decorate(ax, title="chan0 vel", xlab='ticks/s', ylab='time', legend=True)

      ax = plt.subplot(3,2,4)
      plt.plot(self._time_r[s], self.wheel_sp_cpr[s,1], label='sp')
      plt.plot(self._time_r[s], self.enc_vel[s,1], label='meas', alpha=0.5)
      mot1_enc_vel_flt = signal.filtfilt(b, a,  self.enc_vel[s,1])
      plt.plot(self._time_r[s], mot1_enc_vel_flt, label='flt')
      tpu.decorate(ax, title="chan1 vel", xlab='ticks/s', ylab='time', legend=True)

      ax = plt.subplot(3,2,5)
      plt.plot(self._time_r[s], self.mot_a_sp[s,0], label='sp')
      plt.plot(self._time_r[s], self.mot_a_meas[s,0], label='meas', alpha=0.5)
      mot0_a_meas_flt = signal.filtfilt(b, a,  self.mot_a_meas[s,0])
      plt.plot(self._time_r[s], mot0_a_meas_flt, label='flt')
      tpu.decorate(ax, title="chan0 current", xlab='A', ylab='time', legend=True)

      ax = plt.subplot(3,2,6)
      plt.plot(self._time_r[s], self.mot_a_sp[s,1], label='sp')
      plt.plot(self._time_r[s], self.mot_a_meas[s,1], label='meas', alpha=0.5)
      mot1_a_meas_flt = signal.filtfilt(b, a,  self.mot_a_meas[s,1])
      plt.plot(self._time_r[s], mot1_a_meas_flt, label='flt')
      tpu.decorate(ax, title="chan0 current", xlab='A', ylab='time', legend=True)
      plt.show()




def main(filename="/tmp/odrive_vel_step_lifted2.npz"):
   #print (mot_a_meas)
   ds = Dataset(filename)
   ds.plot_chronogram()
 

if __name__ == '__main__':
    main()
