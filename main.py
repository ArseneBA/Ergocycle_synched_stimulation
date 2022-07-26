import time

from pyScienceMode2 import Stimulator as St
from pyScienceMode2 import Channel as Ch

from odrive_control import *
import sensix


def init_motor():
    motor = OdriveEncoderHall()

    return motor


def init_rehastim():
    # Create a list of channels
    list_channels = []

    # Create all channels possible
    channel_1 = Ch.Channel(mode='Single', no_channel=1, amplitude=0, pulse_width=100, inter_pulse_interval=10,
                           name='Biceps_d')
    # channel_2 = Ch.Channel(mode='Single', no_channel=1, amplitude=5, pulse_width=100, inter_pulse_interval=10,
    #                        name='Biceps_g')
    channel_3 = Ch.Channel(mode='Single', no_channel=3, amplitude=0, pulse_width=100, inter_pulse_interval=10,
                           name='Biceps_g')
    # channel_4 = Ch.Channel(mode='Single', no_channel=1, amplitude=0, pulse_width=100, inter_pulse_interval=10,
    #                        name='Biceps_d')
    channel_5 = Ch.Channel(mode='Single', no_channel=5, amplitude=0, pulse_width=100, inter_pulse_interval=10,
                           name='DeltAnt_d')
    channel_6 = Ch.Channel(mode='Single', no_channel=6, amplitude=0, pulse_width=100, inter_pulse_interval=10,
                           name='DeltAnt_g')
    channel_7 = Ch.Channel(mode='Single', no_channel=7, amplitude=0, pulse_width=100, inter_pulse_interval=10,
                           name='Triceps_d')
    channel_8 = Ch.Channel(mode='Single', no_channel=8, amplitude=0, pulse_width=100, inter_pulse_interval=10,
                           name='Triceps_g')

    # Choose which channel will be used
    list_channels.append(channel_1)
    list_channels.append(channel_3)
    list_channels.append(channel_5)
    list_channels.append(channel_6)
    list_channels.append(channel_7)
    list_channels.append(channel_8)

    # Create our object Stimulator
    stimulator = St.Stimulator(list_channels=list_channels, stimulation_interval=1000, port_path='/dev/ttyUSB0')

    stimulator.init_channel()

    return stimulator, list_channels


def set_amp_zero(nw_list_channels):
    for i in range(len(nw_list_channels)):
        nw_list_channels[i].amplitude = 0


def show_list_channels(l_ch):
    for i in range(len(l_ch)):
        print(l_ch[i])


motor = init_motor()
stimulator, list_channels = init_rehastim()

motor.set_speed(1)

stimulation_state = "IDLE"

while 1:
    angle_crank = motor.get_angle_crank()

    # Angle without stimulation
    if (10 <= angle_crank < 20 or 180 <= angle_crank < 220) and stimulation_state != "IDLE":
        set_amp_zero(list_channels)
        stimulator.stop_stimulation()
        stimulation_state = "IDLE"
        print("angle crank", angle_crank)
        print("stimulation_state", stimulation_state)

    if 20 <= angle_crank < 180 and stimulation_state != "TRI_DELT_ANT":
        list_channels[2].amplitude = 5
        list_channels[3].amplitude = 5
        list_channels[4].amplitude = 5
        list_channels[5].amplitude = 5

        stimulator.start_stimulation(upd_list_channels=list_channels)

        stimulation_state = "TRI_DELT_ANT"
        print("angle crank", angle_crank)
        print("stimulation_state", stimulation_state)

    if (220 <= angle_crank < 360 or 0 <= angle_crank < 10) and stimulation_state != "BI_DELT_POS":
        list_channels[0].amplitude = 5
        list_channels[1].amplitude = 5

        stimulator.start_stimulation(upd_list_channels=list_channels)
        stimulation_state = "BI_DELT_POS"
        print("angle crank", angle_crank)
        print("stimulation_state", stimulation_state)

    time.sleep(0.01)
