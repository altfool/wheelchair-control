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
repeated on both platforms. Of course you may only run it in raspberry pi and use computer ssh -X to raspberry pi to
control it.

* First install [ROS](http://wiki.ros.org/ROS/Installation) noetic on raspberry pi (RPi) and your computer.
    * for ros on RPi, you may refer to this [link](http://wiki.ros.org/ROSberryPi/Setting%20up%20ROS%20on%20RaspberryPi)
    * for ros noetic on RPi, you may refer to this [link](https://varhowto.com/install-ros-noetic-raspberry-pi-4/)
* based on this [link](http://wiki.ros.org/ROS/Tutorials/MultipleMachines), set up the connection between PC and RPi
  through LAN or wifi.
    * Suppose PC will be a ros master, for brevity, you can just `export ROS_MASTER_URI=http://<master-ip>:11311` to
      your RPi `~/.bashrc` file (remember to replace master-ip with PC ip adress). Then ros on RPi will know where to
      find the ros master when starting. Remember to `source ~/.bashrc` if you modify it.
    * Once you defined which one is master machine, then you need to start it first with `roscore` or the other node
      will use it's own ip as master uri.
* `git clone https://github.com/altfool/wheelchair-control` on both RPi and computer.

* `cd wheelchair-control/keyboardctl-ros` on both side

* `catkin_make` on both side

* `source devel/setup.bash` on both side

* Now you should have PC running and RPi running (connected to wheelchair) with wheelchair joystick module (original
  JSM) running.
    * on PC, open a terminal and run `roscore` to start ros master
    * on RPi, run `roslaunch wheelchair_control wheelchair_control.launch`.
        * you should hear a song from wheelchair which indicates it's ready.
    * on PC again, open another terminad and run `roslaunch robot_keyboard_control robot_keyboard_control.launch`
        * now you can use arrow keys to control wheelchair, use key 'g' to toggle back to normal mode, use key 'esc' to
          quit keyboard control.
    * we start ros master first, then wheelchair control node, keyboard control node as last so that we won't activate
      keyboard control when we input commands on keyboard for running other nodes.
    * always remember to `source <path-to-keyboardctl-ros/devel/setup.bash>` for new opened terminals.

### Wheelchair vs. Raspberry Pi

* It doesn't matter which one you start first, the wheelchair JSM or RPi.
* But RPi will fake wheelchair JSM commands and control wheelchair. The JSM will be interrupted. So make sure JSM is
  controlling wheelchair when you want to steal control from it by RPi.
* Once the JSM is shut down, RPi can not fake commands any more. So for safety usage, you can directly shut down the JSM
  module. And this will directly stop the wheelchair since it uses auto-brakes.

## Attention

If your wheelchair has a higher moving speed or move to some wrong direction, just press the **down arrow (&darr;)** to
stop it! The joystick on wheelchair won't work in this case (you can shut it down to stop wheelchair as well)! Once the
down arrow pressed, it immediated stop the wheelchair. Please be CAREFUL when you test the code and we are **NOT**
responsible for anything happened.
