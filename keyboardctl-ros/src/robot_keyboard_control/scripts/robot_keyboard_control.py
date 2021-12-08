#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import math

robot_keyboardctl_flag = False
xforward = 0
zrotate = 0


def on_release(key):
    global xforward, zrotate, robot_keyboardctl_flag
    # print('{0} released'.format(key))
    robot_keyboardctl_flag = True  # press any key will enable keyboard ctl, 'g' stop control, esc will quit.
    if key == keyboard.Key.up:
        xforward = min(robot_max_forward_speed, xforward + robot_forward_speed_inctl)
    if key == keyboard.Key.left:
        zrotate = max(-robot_max_rotation_speed, zrotate - robot_rotation_speed_inctl)
    if key == keyboard.Key.right:
        zrotate = min(robot_max_rotation_speed, zrotate + robot_rotation_speed_inctl)
    if key == keyboard.Key.down:
        xforward = 0
        zrotate = 0
    # 'g' and esc will both quit keyboard-control mode. g is temporary, esc is permanent
    if key == keyboard.KeyCode.from_char('g') or key == keyboard.Key.esc:
        robot_keyboardctl_flag = False
        xforward = 0
        zrotate = 0
    rospy.set_param('/robot_keyboard_control_flag', robot_keyboardctl_flag)

    if key == keyboard.Key.esc:
        # Stop listener
        return False

def reset():
    print("shutdown time! reset robot_keyboard_control_flag to False!")
    rospy.set_param('/robot_keyboard_control_flag', False)

def vel_publisher():
    global vel_pub
    vel_pub = rospy.Publisher("~cmd_vel", Twist, queue_size=500)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = xforward
        vel_msg.angular.z = math.radians(zrotate)
        vel_pub.publish(vel_msg)
        rospy.loginfo("published twist info with keyboardctl_flag: {}, linear.x: {}, angular.z: {}".format(
            robot_keyboardctl_flag, xforward, math.radians(zrotate)))
        rate.sleep()
    rospy.on_shutdown(reset)



def main():
    global robot_max_rotation_speed, robot_rotation_speed_inctl, robot_max_forward_speed, robot_forward_speed_inctl
    rospy.init_node("robot_keyboard_control")
    # read & set params for ros param server
    robot_max_rotation_speed = rospy.get_param('~robot_max_rotation_speed', default=90.0)
    robot_rotation_speed_inctl = rospy.get_param('~robot_rotation_speed_inctl', default=5.0)
    robot_max_forward_speed = rospy.get_param('~robot_max_forward_speed', default=3.0)
    robot_forward_speed_inctl = rospy.get_param('~robot_forward_speed_inctl', default=0.2)
    rospy.set_param('/robot_keyboard_control_flag', robot_keyboardctl_flag)

    listener = keyboard.Listener(on_release=on_release)
    listener.start()
    try:
        vel_publisher()
    except rospy.ROSException as e:
        rospy.logerr("vel_publisher call failed: {}".format(e))


if __name__ == '__main__':
    main()
