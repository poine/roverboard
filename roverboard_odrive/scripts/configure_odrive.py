#!/usr/bin/python3
import sys, time
import math, numpy as np

import odrive, fibre, odrive.enums


def config_axis(odrv, axis):
    axis.motor.config.pole_pairs = 15
    axis.motor.config.resistance_calib_max_voltage = 4
    axis.motor.config.requested_current_range = 25 
    axis.motor.config.current_control_bandwidth = 100

    axis.encoder.config.mode = odrive.enums.ENCODER_MODE_HALL
    axis.encoder.config.cpr = 90

    axis.encoder.config.bandwidth = 100
    axis.controller.config.pos_gain = 1
    axis.controller.config.vel_gain = 0.02
    axis.controller.config.vel_integrator_gain = 0.1
    axis.controller.config.vel_limit = 1000
    axis.controller.config.control_mode = odrive.enums.CTRL_MODE_VELOCITY_CONTROL
    
    #odrv.save_configuration()
    #odrv.reboot()
    axis.requested_state = odrive.enums.AXIS_STATE_MOTOR_CALIBRATION
    axis.motor.config.pre_calibrated = True
    time.sleep(5)
    axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(5)
    axis.encoder.config.pre_calibrated = True

    axis.config.startup_closed_loop_control = True
    odrv.save_configuration()
    odrv.reboot()

     

def main():
    app_shutdown_token = fibre.Event()
    odrv = odrive.find_any()
    print("serial: {}".format(odrv.serial_number))
    config_axis(odrv, odrv.axis1
    )
    
if __name__ == '__main__':
    sys.stderr.write("ODrive configuration (using odrive lib version " + odrive.__version__ + ")\n")
    main()
