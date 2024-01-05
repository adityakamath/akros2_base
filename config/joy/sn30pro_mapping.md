8BitDo SN30 Pro 
*WARNING: X-Y and A-B buttons are flipped compared to an Xbox controller*

Android Mode is recommended for Raspberry Pi devices (Some connectivity issues observed in Windows Mode)
Windows Mode is recommended for all other devices
Switch Mode should be used only if the Android and Windows modes don't work

Windows Mode - ctrl+X (haptic feedback available, recommended):
* axes:
  * x: 0 # left joystick left-right
  * y: 1 # left joystick up-down
  * l2: 2 # pressed: 1.0, not pressed: 1.0
  * z: 3 # right joystick left-right
  * rz: 4 # right joystick up-down
  * r2: 5 # pressed: 1.0, not pressed: 1.0
  * left_right: 6 # dpad left-right
  * up_down: 7 # dpad up-down
* buttons:
  * b: 0
  * a: 1
  * y: 2
  * x: 3
  * l1: 4
  * r1: 5
  * select: 6
  * start: 7
  * ljoy: 8
  * rjoy: 9
  * home: 10 # right corner button

Android Mode - ctrl+B (no haptic feedback available - recommended, especially for RPi):
* axes:
  * x: 0 # left joystick left-right
  * y: 1 # left joystick up-down
  * z: 2 # right joystick left-right
  * rz: 3 # right joystick up-down
  * r2: 4 # pressed: -1.0, not pressed: 1.0 - only in bluetooth mode, not shown in wired mode
  * l2: 5 # pressed: -1.0, not pressed: 1.0 - only in bluetooth mode, not shown in wired mode
  * left_right: 6 # dpad left-right, 4 in wired mode
  * up_down: 7 # dpad up-down, 5 in wired mode
* buttons:
  * a: 0
  * b: 1
  * mystery1: 2
  * x: 3
  * y: 4
  * mystery2: 5
  * l1: 6
  * r1: 7
  * l2: 8 # wired mode only, shown in bluetooth mode as unpressed
  * r2: 9 # wired mode only, shown in bluetooth mode as unpressed
  * select: 10
  * start: 11
  * home: 12 # right corner button
  * ljoy: 13
  * rjoy: 14
  * mystery5: 15
 
Switch Mode - ctrl+Y (no haptic feedback available, not recommended):
* axes:
  * x: 0 # left joystick left-right
  * y: 1 # left joystick up-down
  * z: 3 # right joystick left-right
  * rz: 4 # right joystick up-down
  * left_right: 6 # dpad left-right
  * up_down: 7 # dpad up-down
* buttons:
  * b: 0
  * a: 1
  * y: 2
  * x: 3
  * l1: 4
  * r1: 5
  * l2: 6
  * r2: 7
  * select: 8
  * start: 9
  * ljoy: 10
  * rjoy: 11
  * home: 12 # right corner button
  * star: 13 # left corner button
  * mystery1: 14
  * mystery2: 15

MacOS Mode - ctrl+A (does not work with Linux/Windows - not recommended)
