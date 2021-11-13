This folder includes codes for remotely controlling wheelchair with keyboard.
We currently use python to implement it.

For raspberry pi with X display, you may try the file "wheelchair_teleop_key_pynput.py" in which the module pynpus is used and it needs the display envrionment setup.

For raspberry pi without X display, you may try the file "wheelchair_teleop_key_pynput.py" in which the module keyboard is used and it needs only the sudo priority.
(We currently just tested this version. All the codes except keyboard input detection are the same in these 2 files.)