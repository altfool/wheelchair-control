#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import math

wheelchair_keyboardctl_flag = False
xforward = 0
zrotate = 0


def on_release(key):
    global xforward, zrotate, wheelchair_keyboardctl_flag
    # print('{0} released'.format(key))
    wheelchair_keyboardctl_flag = True  # press any key will enable keyboard ctl, esc will release it.
    if key == keyboard.Key.up:
        xforward = min(wheelchair_max_forward_speed, xforward + wheelchair_forward_speed_inctl)
    if key == keyboard.Key.left:
        zrotate = max(-wheelchair_max_rotation_speed, zrotate - wheelchair_rotation_speed_inctl)
    if key == keyboard.Key.right:
        zrotate = min(wheelchair_max_rotation_speed, zrotate + wheelchair_rotation_speed_inctl)
    if key == keyboard.Key.down:
        xforward = 0
        zrotate = 0
    if key == keyboard.Key.esc:
        wheelchair_keyboardctl_flag = False
    rospy.set_param('/keyboard_wheelchair_control', wheelchair_keyboardctl_flag)


def vel_publisher():
    vel_pub = rospy.Publisher("/wheelchair_keyboard_control/cmd_vel", Twist, queue_size=100)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = xforward
        vel_msg.angular.z = math.radians(zrotate)
        vel_pub.publish(vel_msg)
        rospy.loginfo("published twist info with keyboardctl_flag: {}, linear.x: {}, angular.z: {}".format(wheelchair_keyboardctl_flag, xforward, math.radians(zrotate)))
        rate.sleep()


def main():
    global wheelchair_max_rotation_speed, wheelchair_rotation_speed_inctl, wheelchair_max_forward_speed, wheelchair_forward_speed_inctl
    rospy.init_node("wheelchair_keyboard_control")
    # read & set params for ros param server
    wheelchair_max_rotation_speed = rospy.get_param('~wheelchair_max_rotation_speed', default=90.0)
    wheelchair_rotation_speed_inctl = rospy.get_param('~wheelchair_rotation_speed_inctl', default=5.0)
    wheelchair_max_forward_speed = rospy.get_param('~wheelchair_max_forward_speed', default=3.0)
    wheelchair_forward_speed_inctl = rospy.get_param('~wheelchair_forward_speed_inctl', default=0.2)
    rospy.set_param('/keyboard_wheelchair_control', wheelchair_keyboardctl_flag)

    listener = keyboard.Listener(on_release=on_release)
    listener.start()
    try:
        vel_publisher()
    except rospy.ROSException as e:
        rospy.logerr("vel_publisher call failed: {}".format(e))


if __name__ == '__main__':
    main()
