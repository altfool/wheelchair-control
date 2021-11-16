This folder includes codes for remotely controlling wheelchair with keyboard.
We currently use python to implement it.

### How it works
For raspberry pi **with** X display, you may try the file "wheelchair_teleop_key_pynput.py" in which the module pynput is used and it needs the display envrionment setup.
(We currently just tested this version. All the codes except keyboard input detection are the same in these 2 files.)

Use "ssh -X" to connect to remote RPi.

For raspberry pi **without** X display, you may try the file "wheelchair_teleop_key_keyboard.py" in which the module keyboard is used and it needs the sudo priority.

### keyboard control
arrow up (&uarr;): increase forward speed (every time increase 5, Min: 0, Max: 100)

arrow left (&larr;) & right (&rarr;): adjust rotation.

    left: increase counter-clockwise rotation speed (every time increase 5)
    
    right: increase clockwise rotation speed (every time increase 5)
    
    if press right when rotating counter-clockwise, it will decrease the rotation.
    
    The backend is a signed value (Min:-100, Max:100) to control rotation speed and direction.
    
arrow down (&darr;): immediate stop

    set every motion value to 0; Since the wheelchair will automatically apply the brake, so it will stop immediately.
    

key 'u' & 'i':

    key 'u': decrease power level (Min: 0%)
    
    key 'i': increase power level (Max: 100%)
    
    When running script, the initial power level of wheelchair is set to minimum (i.e. 0%). But just care, it will move still fast.
    

### Attention
  If your wheelchair has a higher moving speed or move to some wrong direction, just press the **down arrow (&darr;)** to stop it! The joystick on wheelchair won't work in this case! Once the down arrow pressed, it immediated stop the wheelchair. Please be CAREFUL when you test the code and we are not RESPONSIBLE for anything happened.
