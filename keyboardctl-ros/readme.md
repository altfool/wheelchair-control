This is the ros version for keyboard control of wheelchair.

## Package: wheelchair_control

This is the package actually controlling wheelchair. Please adjust the configuration files
inside [config](/keyboardctl-ros/src/wheelchair_control/config) based on your own wheelchair info.

The wheelchair we use has 5 speed levels and we manually define them based on the max-speed from wheelchair's manual (
since we don't have any details about this). The same thing happens to the wheelchair max rotation speed. You can set
higher values if your wheelchair acts faster than you expect. This ros node accepts /cmd_vel and covert it to wheelchair
rotation Xx, speed Yy & speed level. So if wheelchair max speed is set higher, the calculated speed from /cmd_vel will
becomes smaller.

Income: /cmd_vel (x/forward speed, z-rotation)

Outcome: CAN frame to control wheelchair (Xx, Yy, speed-level)

## Package: robot_keyboard_control

This is the package listening to keyboard events and send speed commands (/cmd_vel) to robot (here is node
wheelchair_control). If necessary, you may adjust the arguments
inside [launch file](/keyboardctl-ros/src/robot_keyboard_control/launch/robot_keyboard_control.launch).

Income: key-press (esc/g/left/right/up/down)

Outcome: cmd_vel

### keyboard control

Press any key will enable keyboard control. Arrow keys are used to control direction. Key 'g' is used to toggle back
normal mode. Key 'esc' is used to quit keyboard control.

* arrow up (&uarr;): increase forward speed (increase 0.1 m/s per keypress)

* arrow left (&larr;) & right (&rarr;): adjust rotation.

    * left: increase counter-clockwise rotation speed (increase 5 degree per keypress)

    * right: increase clockwise rotation speed (increase 5 degree per keypress)

    * if press right when rotating counter-clockwise, it will decrease the rotation.

    * The backend is a signed value "zrotate" (Min:-90, Max:90) to control rotation speed and direction.

* arrow down (&darr;): immediate stop

    * set 0s to all values

* key 'g':

    * disable keyboard control and switch back to system control (not existing here).
    * this key is temporary, it just reset all values to 0s and disable keyboard control.

* key 'esc':

    * disable keyboard permanently and switch back to system control (not existing here).
    * this key is permanently, it will reset all values to 0s and quit the keypress listener.

## Steps to install

Here we provide a way to communicate PC with raspberry pi (RPi) on wheelchair through ROS. So most steps will be
repeated on both platforms. Of course you may only run it in raspberry pi and use computer ssh -X to raspberry to
control it.

* First install [ROS](http://wiki.ros.org/ROS/Installation) on raspberry pi (RPi) and your computer.

* `git clone https://github.com/altfool/wheelchair-control` on both RPi and computer.

* `cd wheelchair-control/keyboardctl-ros` on both side

* `catkin_make` on both side

[comment]: <> (* Git clone this repo into raspberry pi &#40;RPi&#41;.)

[comment]: <> (* Find raspberry pi's ip address &#40;see below&#41;.)

[comment]: <> (* Plug RPi into R-net hub to connect wheelchair. Start wheelchair joystick module &#40;the original JSM&#41;.)

[comment]: <> (* On computer, use `ssh -X <username>@<ip>` to connect RPi. Remember to replace the username and ip with your own about)

[comment]: <> (  RPi.)

[comment]: <> (* run `python3 <path-to-this-folder>/wheelchair_teleop_key_pynput.py` and you should hear a song from wheelchair. And)

[comment]: <> (  you can use keyboard to control the wheelchair. Please read the following instructions **first** before you actually)

[comment]: <> (  work on it.)

## Attention

If your wheelchair has a higher moving speed or move to some wrong direction, just press the **down arrow (&darr;)** to
stop it! The joystick on wheelchair won't work in this case! Once the down arrow pressed, it immediated stop the
wheelchair. Please be CAREFUL when you test the code and we are **NOT** responsible for anything happened.
