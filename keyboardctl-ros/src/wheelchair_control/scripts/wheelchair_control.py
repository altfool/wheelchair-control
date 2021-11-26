#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from can2RNET import *
import math

speed_level = 1  # initialize power level: lvl1 (total 5 levels [0,25,50,75,100])
speed_level_inctl = 25  # increase 25% per level
speed_level_changed = True

# wheelchair control code
drive_mode_code = "02000100#"  # check your own wheelchair mode and replace it.
frame_jsm_induce_error = "0c000000#"  # code to JSM induce error so that we can take over the wheelchair
can_socket = opencansocket(0)  # comment out if only test keyboards

def wheelchairctl_callback(msg: Twist, keyboardctl_flag=False):
    global speed_level, speed_level_changed
    wheelchair_keyboard_control_flag = rospy.get_param('/robot_keyboard_control_flag', default=False)
    if wheelchair_keyboard_control_flag == keyboardctl_flag:
        xforward = msg.linear.x
        zrotate = math.degrees(msg.angular.z)
        # find correct Xx, Yy, and speed-level (Yy:forward, Xx:rotate)
        for idx in range(1, 6):
            if xforward <= wheelchair_speed_dict['lvl{0}'.format(idx)]:
                # speed located in current range, idx shows speed-level
                speed_level_changed = (speed_level != idx)
                speed_level = idx
                break
        Yy = min(int(xforward / wheelchair_speed_dict['lvl{0}'.format(speed_level)] * 100),
                 100)  # in case linear.x too large, truncate it to 100
        Xx = int(zrotate / wheelchair_max_rotation_speed * 100)
        if abs(Xx) > 100:
            Xx = math.copysign(100, Xx)
        rospy.loginfo("keyboardctl: {}, Speed-Level: {}, Xx: {}, Yy: {}".format(keyboardctl_flag, speed_level, Xx, Yy))
        # send wheelchair cmds based on Xx, Yy, and speed-level
        if speed_level_changed:
            RNETsetSpeedRange(can_socket, (speed_level - 1) * speed_level_inctl)
            speed_level_changed = False
        if Xx < 0:
            Xx_cmd = dec2hex(0x100+Xx, 2)   # ~xlevel + 1 = 0xFF-abs(xlevel)+0x1
        else:
            Xx_cmd = dec2hex(Xx, 2)
        drive_cmd = drive_mode_code + Xx_cmd + dec2hex(Yy, 2)
        cansend(can_socket, drive_cmd)        # comment out if only test keyboards


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


def main():
    global speed_level_changed, wheelchair_speed_dict, wheelchair_max_rotation_speed
    RNETplaysong(can_socket)  # comment out if only test keyboards
    for _ in range(5):
        cansend(can_socket, frame_jsm_induce_error)  # send in less than 1ms to induce JSM error
    if speed_level_changed:
        RNETsetSpeedRange(can_socket, (speed_level - 1) * speed_level_inctl)
        speed_level_changed = False

    # ros node init
    rospy.init_node('wheelchair_control')

    # read params from ros param server
    wheelchair_max_rotation_speed = rospy.get_param('~wheelchair_max_rotation_speed', 100)
    wheelchair_speed_dict = {}
    for idx in range(1, 6):
        speed_str = rospy.get_param('~wheelchair_max_speed_lvl{0}'.format(idx), default=4.0 / 5 * idx)
        wheelchair_speed_dict['lvl{0}'.format(idx)] = eval(speed_str) if isinstance(speed_str, str) else speed_str
        # rospy.loginfo("wheelchair_max_speed_lvl{0}: {1}".format(idx, wheelchair_speed_dict['lvl{0}'.format(idx)]))

    rospy.Subscriber("/robot_keyboard_control/cmd_vel", Twist, wheelchairctl_callback,
                     callback_args=True)  # if keyboardctl enabled, will directly control wheelchair through this
    rospy.Subscriber("/robot_system_control/cmd_vel", Twist, wheelchairctl_callback,
                     callback_args=False)  # else, control wheelchair through system
    rospy.spin()


if __name__ == '__main__':
    main()
