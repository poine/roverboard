#!/usr/bin/python3
import sys
from fibre import Logger, Event
import odrive

sys.stderr.write("ODrive test (using odrive lib version " + odrive.__version__ + ")\n")
 
app_shutdown_token = Event()

my_odrive = odrive.find_any(path="usb:003:030", serial_number=None,
                            search_cancellation_token=app_shutdown_token,
                            channel_termination_token=app_shutdown_token)
#print(my_odrive._remote_attributes.items())
print("serial: {}".format(my_odrive.serial_number))
print("encoders pos: {} {}".format(my_odrive.axis0.encoder.pos_estimate, my_odrive.axis1.encoder.pos_estimate))
print("encoders vel: {} {}".format(my_odrive.axis0.encoder.vel_estimate, my_odrive.axis1.encoder.vel_estimate))
