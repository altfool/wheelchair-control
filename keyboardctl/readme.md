This folder includes codes for remotely controlling wheelchair with keyboard.
We currently use python to implement it.

For raspberry pi **with** X display, you may try the file "wheelchair_teleop_key_pynput.py" in which the module pynput is used and it needs the display envrionment setup.
(We currently just tested this version. All the codes except keyboard input detection are the same in these 2 files.)

Use "ssh -X" to connect to remote RPi.

For raspberry pi **without** X display, you may try the file "wheelchair_teleop_key_keyboard.py" in which the module keyboard is used and it needs the sudo priority.
