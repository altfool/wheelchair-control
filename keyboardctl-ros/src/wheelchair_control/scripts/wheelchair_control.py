#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# default value
# keyboard_wheelchair_control = False
wheelchair_speed_dict = {}
for idx in range(1, 5):
    wheelchair_speed_dict['lvl{0}'.format(idx)] = 4.0 / 5 * idx  # lvl1 to lvl4


def wheelchairctl_callback(msg: Twist, keyboardctl_flag=False):
    keyboard_wheelchair_control_flag = rospy.get_param('/keyboard_wheelchair_control')
    if keyboard_wheelchair_control_flag == keyboardctl_flag:
        print("keyboad_wheelchair_control_flag: {}".format(keyboardctl_flag))
        pass

def main():
    global wheelchair_speed_dict

    # ros node init
    rospy.init_node('wheelchair_control')

    # read parameters from param server
    for idx in range(1, 5):
        speed_str = rospy.get_param('/wheelchair_max_speed_lvl{0}'.format(idx))
        wheelchair_speed_dict['lvl{0}'.format(idx)] = eval(speed_str) if isinstance(speed_str, str) else speed_str
        rospy.loginfo("wheelchair_max_speed_lvl{0}: {1}".format(idx, wheelchair_speed_dict['lvl{0}'.format(idx)]))

    rospy.Subscriber("/wheelchair_keyboard_control/cmd_vel", Twist, wheelchairctl_callback, callback_args=True)   # if keyboardctl enabled, will directly control wheelchair through this
    rospy.Subscriber("/wheelchair_system_control/cmd_vel", Twist, wheelchairctl_callback, callback_args=False)       # else, control wheelchair through system

    rospy.spin()


if __name__ == '__main__':
    main()
