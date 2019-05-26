#!/usr/bin/env python

import can

can_interface = 'can1'
bus = can.interface.Bus(can_interface, bustype='socketcan_native')
message = bus.recv()
print(message)
