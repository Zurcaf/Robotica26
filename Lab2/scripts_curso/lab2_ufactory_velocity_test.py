#
# script to test joint velocity control
#
# adapted from   https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/example/wrapper/common/2000-joint_velocity_control.py
#


import os
import sys
import time

from xarm.wrapper import XArmAPI


arm = XArmAPI('192.168.1.158')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)

# if the robot has the gripper installed, sending it home
# can lead to a colision 
#arm.move_gohome(wait=True)

# set joint velocity control mode
arm.set_mode(4)
arm.set_state(0)
time.sleep(1)

count = 6
while count > 0:
    count -= 1
    #arm.vc_set_joint_velocity([-50, 0, 0, 0, 0, 0])
    #arm.vc_set_joint_velocity([0, -10, 0, 0, 0, 0])
    #arm.vc_set_joint_velocity([0, 0, -10, 0, 0, 0])
    #arm.vc_set_joint_velocity([0, 0, 0, -10, 0, 0])
    arm.vc_set_joint_velocity([0, 0, 0, 0, -10, 0])
    time.sleep(2)
    #arm.vc_set_joint_velocity([50, 0, 0, 0, 0, 0])
    #arm.vc_set_joint_velocity([0, 10, 0, 0, 0, 0])
    #arm.vc_set_joint_velocity([0, 0, 10, 0, 0, 0])
    #arm.vc_set_joint_velocity([0, 0, 0, 10, 0, 0])
    arm.vc_set_joint_velocity([0, 0, 0, 0, 10, 0])
    time.sleep(2)

# stop
arm.vc_set_joint_velocity([0, 0, 0, 0, 0, 0])

arm.disconnect()
