#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, George Kouros
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
#  * Neither the name of the copyright holder nor the names of its
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

'''
four_wheel_steer_teleop.py:
    A ros keyboard teleoperation script for four wheel steer robots
'''

__author__ = 'George Kouros'
__license__ = 'BSD'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import thread
from numpy import clip
from math import sin, cos, tan

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09',
    'mode'  : '\x6d',
    'CW'     : '\x7A',
    'CCW'     : '\x78',}
key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0),
    '\x6d' : ( 0.0 , 0.0),
    '\x7A' : ( 1.0 , 0.0),
    '\x78' : ( -1.0 , 0.0),}



class FourWheelSteerTeleop:

    def __init__(self):
        # load max speed, max steering angle, wheelbase, cmd topic
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.wheelbase = rospy.get_param('~wheelbase', 0.1)
        self.max_speed = rospy.get_param('~max_speed', 2)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 0.4)

        self.mode = 0  # 0: counter steer mode, 1: crab steer mode
        self.speed_range = [-float(self.max_speed), float(self.max_speed)]
        self.steering_angle_range = [-float(self.max_steering_angle),
                                     float(self.max_steering_angle)]
        self.speed = 0
        self.rotspeed = 0
        self.steering_angle = 0
        self.drive_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        cmd_msg = Twist()        
        cmd_msg.linear.x = self.speed * cos(self.steering_angle * self.mode)
        cmd_msg.linear.y = self.speed * sin(self.steering_angle * self.mode)
        cmd_msg.linear.z = self.rotspeed         
        cmd_msg.angular.z = 2 * self.speed \
                * cos(self.steering_angle * self.mode) \
                * tan(self.steering_angle) * (1 - self.mode) / self.wheelbase
        self.drive_pub.publish(cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r****************************************')
        rospy.loginfo('\x1b[1M\r↑/↓: increase/decrease speed')
        rospy.loginfo('\x1b[1M\r←/→: increase/decrease steering angle')
        rospy.loginfo('\x1b[1M\rm: switch steering mode')
        rospy.loginfo('\x1b[1M\rspace: brake')
        rospy.loginfo('\x1b[1M\rtab: reset steering angle')
        rospy.loginfo('\x1b[1M\rq: exit')
        rospy.loginfo('\x1b[1M\r****************************************')
        rospy.loginfo('\x1b[1M\r\033[34;1mMode: \033[31;1m%s',
                      'crab steer'*self.mode+'counter steer'*abs(self.mode-1))
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)      
        rospy.loginfo('\x1b[1M\rot '
                      '\033[34;1mZspd: \033[32;1m%0.2f m/s',
                      self.rotspeed)                   

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            print(key)
            if key in key_bindings.keys():                
                if key == control_keys['space']:
                    self.speed = 0.0
                    self.rotspeed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                    self.rotspeed = 0.0
                elif key == control_keys['mode']:
                    self.rotspeed = 0.0
                    self.mode = abs(self.mode-1)
                        #### 
                elif key == control_keys['CW']:
                    print("CW steering")
                    self.steering_angle = 0.0
                    self.speed = 0.0
                    if (self.rotspeed <self.max_speed):
                        self.rotspeed += 0.1 
                elif key == control_keys['CCW']:
                    print("CCW steering")
                    self.steering_angle = 0.0
                    self.speed = 0.0
                    if (self.rotspeed > -self.max_speed):
                        self.rotspeed -= 0.1            
                            ####                  
                else:  
                    if(self.rotspeed==0):                  
                        self.speed = \
                                self.speed + key_bindings[key][0] * \
                                self.max_speed / 50
                        self.steering_angle = \
                                self.steering_angle + key_bindings[key][1] * \
                                self.max_steering_angle / 50
                        self.speed = clip(
                            self.speed, self.speed_range[0], self.speed_range[1])
                        self.steering_angle = clip(
                            self.steering_angle,
                            self.steering_angle_range[0],
                            self.steering_angle_range[1])                            
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            

            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Braking, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        cmd_msg = Twist()
        cmd_msg.linear.x = 0
        cmd_msg.linear.y = 0
        cmd_msg.angular.z = 0
        self.drive_pub.publish(cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('four_wheel_steer_teleop')
    teleop = FourWheelSteerTeleop()
