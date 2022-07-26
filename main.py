import time

import nidaqmx
import multiprocessing as mp

from pyScienceMode2 import Stimulator as St
from pyScienceMode2 import Channel as Ch

from odrive_control import *

def max_min_moy_milli_amp_value():
    task = nidaqmx.Task()

    # Give an output that change when the crank is moved [-0.005 ;-0.020] change of signe if the crank is rotated
    # in the other direction (measure current). It's the right one
    task.ai_channels.add_ai_current_chan("Dev1/AI14")

    task.start()

    max_val = []
    min_val = []
    for i in range(10):
        val2 = abs(task.read()) * 1000
        print(val2)
        max_val.append(10)
        min_val.append(10)

        while 6.9 > val2 or val2 > 7.0:
            if val2 > max_val[i]:
                max_val[i] = val2
            if val2 < min_val[i]:
                min_val[i] = val2
            val2 = abs(task.read()) * 1000

        while val2 > 6.8:
            val2 = abs(task.read()) * 1000

    max_max_val = 0
    min_min_val = 10

    max_val_moy = 0
    min_val_moy = 0

    for i in range(10):
        max_val_moy += max_val[i]
        min_val_moy += min_val[i]

        if max_val[i] > max_max_val:
            max_max_val = max_val[i]

        if min_val[i] < min_min_val:
            min_min_val = min_val[i]

    max_val_moy = max_val_moy / 10
    min_val_moy = min_val_moy / 10

    print("Max moy", max_val_moy, "Max", max_max_val)
    print("Min moy", min_val_moy, "Min", min_min_val)

    task.stop()
    task.close()


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
    stimulator = St.Stimulator(list_channels=list_channels, stimulation_interval=1000, port_path='COM5')

    stimulator.init_channel()

    return stimulator, list_channels


def set_amp_zero(nw_list_channels):
    for i in range(len(nw_list_channels)):
        nw_list_channels[i].amplitude = 0


def get_angle(task):
    value_milli_amp = abs(task.read()) * 1000

    # 20.07 is the maximum value corresponding to 0°, 5.49 corresponding to 360° (in order to be in trigonometric sens)
    # To be logical the angle increase when the bike is going forward
    return abs(((value_milli_amp - 5.49) * 360 / (20.07 - 5.49)) - 360)


def get_and_process_angle(queue_stim, stim_angle_event):
    task = nidaqmx.Task()
    task.ai_channels.add_ai_current_chan("Dev1/AI14")
    task.start()

    stimulation_state = "BEGINNING"
    state_changed = False

    while 1:
        angle_crank = get_angle(task)

        if (10 <= angle_crank < 20 or 180 <= angle_crank < 220) and stimulation_state != "IDLE":
            stimulation_state = "IDLE"
            state_changed = True

        elif 20 <= angle_crank < 180 and stimulation_state != "TRI_DELT_ANT":
            stimulation_state = "TRI_DELT_ANT"
            state_changed = True

        elif (220 <= angle_crank < 360 or 0 <= angle_crank < 10) and stimulation_state != "BI_DELT_POS":
            stimulation_state = "BI_DELT_POS"
            state_changed = True

        if state_changed:
            queue_stim.put(stimulation_state)
            stim_angle_event.set()
            print("angle crank", angle_crank)
            print("stimulation_state", stimulation_state)
            state_changed = False
        time.sleep(0.001)


def stimulation(queue_stim, stim_angle_event):
    stimulator, list_channels = init_rehastim()

    while 1:
        stim_angle_event.wait()

        stimulation_state = queue_stim.get()

        if stimulation_state == "IDLE":
            set_amp_zero(list_channels)
            stimulator.stop_stimulation()

        if stimulation_state == "TRI_DELT_ANT":
            list_channels[2].amplitude = 5
            list_channels[3].amplitude = 5
            list_channels[4].amplitude = 5
            list_channels[5].amplitude = 5
            stimulator.start_stimulation(upd_list_channels=list_channels)

        if stimulation_state == "BI_DELT_POS":
            list_channels[0].amplitude = 5
            list_channels[1].amplitude = 5
            stimulator.start_stimulation(upd_list_channels=list_channels)

        stim_angle_event.clear()


if __name__ == '__main__':
    queue_stimulation_state = mp.Queue()
    new_stim_angle_event = mp.Event()
    angle_proc = mp.Process(name="get_angle",
                            target=get_and_process_angle,
                            args=(queue_stimulation_state, new_stim_angle_event))
    stim_proc = mp.Process(name="send_stim",
                           target=stimulation,
                           args=(queue_stimulation_state, new_stim_angle_event))
    angle_proc.start()
    stim_proc.start()
    angle_proc.join()
    stim_proc.join()
