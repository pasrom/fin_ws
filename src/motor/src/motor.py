#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import UInt64MultiArray
from std_msgs.msg import Float64
import explorerhat

leftwheel = explorerhat.motor.one
rightwheel = explorerhat.motor.two
leftInc = explorerhat.input.one
righInc = explorerhat.input.two

leftCtr = 0
rightCtr = 0
setSpeed = 0

def countUpLeft(input):
    global leftCtr
    leftCtr = leftCtr + input.read()

def countUpRight(input):
    global rightCtr
    rightCtr = rightCtr + input.read()

def callbackLeftWheel(data):
    global leftwheel
    if data.data > 0:
        leftwheel.forwards(data.data)
    else:
        leftwheel.backwards(data.data*-1)
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.data[0])

def callbackRightWheel(data):
    global rightwheel
    if data.data > 0:
        rightwheel.forwards(data.data)
    else:
        rightwheel.backwards(data.data*-1)
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.data[1])

def callbackWheelLR(data):
    global leftwheel
    if data.data[0] > 0:
        leftwheel.forwards(data.data[0])
    else:
        leftwheel.backwards(data.data[0]*-1)
    if data.data[1] > 0:
        rightwheel.forwards(data.data[1])
    else:
        rightwheel.backwards(data.data[1]*-1)
    rospy.loginfo(rospy.get_caller_id() + 'I heard:\tLeft %d\tRight %d', data.data[0], data.data[1])

def callbackSetMotors(data):
    global leftwheel
    global rightwheel
    global setSpeed
    leftSpeed = setSpeed + data.data/2
    rightSpeed = setSpeed - data.data/2
    if setSpeed == 0:
        leftSpeed = 0
        rightSpeed = 0
    if leftSpeed > 0:
        if leftSpeed > 100:
            leftSpeed = 100
        leftwheel.forwards(leftSpeed)
    else:
        if leftSpeed < -100:
            leftSpeed = -100
        leftwheel.backwards(leftSpeed*-1)
    if rightSpeed > 0:
        if rightSpeed > 100:
            rightSpeed = 100
        rightwheel.forwards(rightSpeed)
    else:
        if rightSpeed < -100:
            rightSpeed = -100
        rightwheel.backwards(rightSpeed*-1)
    rospy.loginfo(rospy.get_caller_id() + 'From PID: %d', data.data)

def callbackSetVelocity(data):
    global setSpeed
    setSpeed = data.data

def talker():
    pub_Ink = rospy.Publisher('robot/inkLR', UInt64MultiArray, queue_size=10)
    #pub_RInk = rospy.Publisher('robot/rightInk', Int16, queue_size=10)
    

    rospy.Subscriber('robot/leftWheel', Int16, callbackLeftWheel)
    rospy.Subscriber('robot/rightWheel', Int16, callbackRightWheel)
    #rospy.Subscriber('robot/setMotors', Float64, callbackSetMotors)
    #rospy.Subscriber('robot/setVelocity', Int16, callbackSetVelocity)
    leftInc.on_high(countUpLeft)
    righInc.on_high(countUpRight)
    incLR = UInt64MultiArray()
    incLR.data = [0,0]
    rospy.init_node('robot', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        incLR.data = [leftCtr,rightCtr]
        pub_Ink.publish(incLR)
        #pub_RInk.publish(rightCtr)
        #pub_lInk.publish(leftCtr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
