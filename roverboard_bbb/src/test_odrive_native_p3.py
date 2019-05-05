#!/usr/bin/python3

from fibre import Logger, Event
import odrive

app_shutdown_token = Event()

my_odrive = odrive.find_any(path="usb", serial_number="306739703235",
                            search_cancellation_token=app_shutdown_token,
                            channel_termination_token=app_shutdown_token)
