#
# Test script (adapted from http://github.com/xArm-Developer/xArm-Python-SDK)
#
# Install from pypi
#   pip install xarm-python-sdk

#!/usr/bin/env python3

import time
from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.158')   #the current IP of the robot
time.sleep(0.5)
arm.set_mode(0)
arm.set_state(0)
arm.reset(wait=True)

#Lite6
#Move to pick the object
arm.open_lite6_gripper()
time.sleep(0.5)
arm.set_position(*[300, 0, 300, 180, 0, 0], wait=True)
arm.set_position(*[300, 0, 200, 180, 0, 0], wait=True)
arm.close_lite6_gripper()
time.sleep(0.5)

#Set tcp payload
arm.set_tcp_load(0.3, [0, 0, 30])
arm.set_state(0)

#Move to place the object
arm.set_position(*[300, 0, 300, 180, 0, 0], wait=True)
arm.set_position(*[300, -150, 300, 180, 0, 0], wait=True)
arm.set_position(*[300, -150, 200, 180, 0, 0], wait=True)
arm.open_lite6_gripper()
time.sleep(0.5)
arm.set_tcp_load(0, [0, 0, 30])
arm.set_state(0)
arm.set_position(*[300, -150, 300, 180, 0, 0], wait=True)
arm.close_lite6_gripper()
arm.stop_lite6_gripper()
