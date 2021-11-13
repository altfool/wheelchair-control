# for key release detection: https://stackoverflow.com/questions/52576914/detect-key-release-while-using-keyboard-module

from keyboard import keyboard
from can2RNET import *
from time import time
import threading

# Xx level (rotation): [-10, -9, -8, ..., -1, 0, 1, 2, ..., 9, 10], the interval decided by increment.
# negative value means counter-clock-wise rotate, postive value means clock-wise rotate
_Xx_LEVEL_MIN = -100
_Xx_LEVEL_MAX = 100
_Xx_INCREMENT = 5
# Yy level (forward speed): [0, 1, 2, ..., 8, 9, 10]
_Yy_LEVEL_MIN = 0  # temporarily no use
_Yy_LEVEL_MAX = 100
_Yy_INCREMENT = 5
# power level: [0,25%,50%,75%,100%] totally 5 levels on wheelchair
# keep interval as 25 no changed.
_POWER_LEVEL_MIN = 0
_POWER_LEVEL_MAX = 100
_POWER_LEVEL_INCREMENT = 25

msg_time_interval = 0.01
drive_mode_code = "02000100#"   # check your own wheelchair mode and replace it.
frame_jsm_induce_error = "0c000000#" # code to JSM induce error so that we can take over the wheelchair


def keyDetect():
    global xlevel, ylevel, power_level, power_change_flag
    keypress_up = False
    keypress_right = False
    keypress_left = False
    keypress_down = False
    keypress_u = False
    keypress_i = False
    while True:
        if keypress_up and not keyboard.is_pressed('up'):
            # print("up is released")
            ylevel = min(_Yy_LEVEL_MAX, ylevel + _Yy_INCREMENT)
            keypress_up = False
        if not keypress_up and keyboard.is_pressed('up'):
            keypress_up = True
        if keypress_right and not keyboard.is_pressed('right'):
            # print("right is released")
            xlevel = min(_Xx_LEVEL_MAX, xlevel + _Xx_INCREMENT)
            keypress_right = False
        if not keypress_right and keyboard.is_pressed('right'):
            keypress_right = True
        if keypress_left and not keyboard.is_pressed('left'):
            # print("left is released")
            xlevel = max(_Xx_LEVEL_MIN, xlevel - _Xx_INCREMENT)
            keypress_left = False
        if not keypress_left and keyboard.is_pressed('left'):
            keypress_left = True
        if keypress_down and not keyboard.is_pressed('down'):
            # print("down is released")
            # reset joystick to 0s
            xlevel = 0
            ylevel = 0
            keypress_down = False
        if not keypress_down and keyboard.is_pressed('down'):
            keypress_down = True
        if keypress_u and not keyboard.is_pressed('u'):
            print("u is released")
            tmp_power_level = max(_POWER_LEVEL_MIN, power_level - _POWER_LEVEL_INCREMENT)
            if tmp_power_level != power_level:
                power_level = tmp_power_level
                power_change_flag = True
            keypress_u = False
        if not keypress_u and keyboard.is_pressed('u'):
            keypress_u = True
        if keypress_i and not keyboard.is_pressed('i'):
            print("i is released")
            tmp_power_level = min(_POWER_LEVEL_MAX, power_level + _POWER_LEVEL_INCREMENT)
            if tmp_power_level != power_level:
                power_level = tmp_power_level
                power_change_flag = True
            keypress_i = False
        if not keypress_i and keyboard.is_pressed('i'):
            keypress_i = True
        if keyboard.is_pressed('esc'):
            break

def RNETshortBeep(cansocket):
    cansend(cansocket, "181c0100#0260000000000000")

# Play little song
def RNETplaysong(cansocket):
    cansend(cansocket, "181C0100#2056080010560858")
    sleep(.77)
    cansend(cansocket, "181C0100#105a205b00000000")

# Set speed_range: 0% - 100%
def RNETsetSpeedRange(cansocket, speed_range):
    if speed_range >= 0 and speed_range <= 0x64:
        cansend(cansocket, '0a040100#' + dec2hex(speed_range, 2))
    else:
        print('Invalid RNET SpeedRange: ' + str(speed_range))

def dec2hex(dec, hexlen):
    hex_str = hex(dec)[2:]
    lgth_hex = len(hex_str)
    assert hexlen >= lgth_hex, "hexlen smaller than hex number string!"
    return '0' * (hexlen - lgth_hex) + hex_str

if __name__ == '__main__':
    global xlevel, ylevel, power_change_flag, power_level
    can_socket = opencansocket(0)  # comment out if only test keyboards
    RNETplaysong(can_socket)  # comment out if only test keyboards
    for _ in range(5):
        cansend(can_socket, frame_jsm_induce_error) # send in less than 1ms to induce JSM error

    power_level = _POWER_LEVEL_MIN
    power_change_flag = True
    xlevel = 0
    ylevel = 0
    key_detect_thread = threading.Thread(target=keyDetect, args=(), daemon=True)
    key_detect_thread.start()
    nexttime = time() + msg_time_interval
    while True:
        if power_change_flag:
            RNETsetSpeedRange(can_socket, power_level)  # comment out if only test keyboards
            power_change_flag = False
        print("PowerLevel: {}, Xx: {}, Yy: {}".format(power_level, xlevel, ylevel))
        if xlevel < 0:
            xlevel_cmd = dec2hex(0x100 + xlevel, 2)  # ~xlevel + 1 = 0xFF-abs(xlevel)+0x1
        else:
            xlevel_cmd = dec2hex(xlevel, 2)
        drive_cmd = drive_mode_code + xlevel_cmd + dec2hex(ylevel, 2)
        print("drive cmd: {}".format(drive_cmd))
        cansend(can_socket, drive_cmd)  # comment out if only test keyboards
        t_now = time()
        if t_now < nexttime:
            print("will sleep a while")
            sleep(nexttime - t_now)
            nexttime += msg_time_interval
        else:
            print("can not sleep, not enough time")
            nexttime = t_now + msg_time_interval