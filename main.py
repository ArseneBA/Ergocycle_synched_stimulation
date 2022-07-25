import odrive
from odrive.enums import *

from pyScienceMode2 import Stimulator as St
from pyScienceMode2 import Channel as Ch

from odrive_control import *

CONFIGURATION = False

motor = OdriveEncoderHall()

if CONFIGURATION:
    motor.configuration()

else:
    motor.set_speed(5)
    motor.print_values()

    while 1:
        angle = motor.get_angle()


        if angle == 10:  # Check is the value can be an equal or a window
            pass
