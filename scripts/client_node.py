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

## Simple demo of a rospy service client that calls a service to add
## two integers. 

import sys
import os

import rospy

# imports the OdomCircle service 
from robot_ping.srv import OdomCircleCommand
from robot_ping.srv import MoveBaseSquareCommand

## execute the circle using the OdomCirle service
def initial_circle(vel, sense):

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the odom_circle_server service is available
    # you can optionally specify a timeout
    rospy.wait_for_service('odom_circle')
    
    try:
        # create a handle to the add_two_ints service
        odom_circle = rospy.ServiceProxy('odom_circle', OdomCircleCommand)
        
        print "Requesting %s+%s"%(vel, sense)
        
        # formal style
        resp = odom_circle.call(vel, sense)
        print resp.completed
        if not resp.completed == True:
            raise Exception("odom circle failure")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return
    
def initial_square(vel, sense):
    rospy.wait_for_service('move_base_square')
    try:
        # create a handle to the add_two_ints service
        move_base_square = rospy.ServiceProxy('move_base_square', MoveBaseSquareCommand)
        
        print "Requesting %s+%s"%(vel, sense)
        
        # formal style
        resp = move_base_square.call(vel, sense)
        print resp.completed
        if not resp.completed == True:
            raise Exception("move base failure")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    return

def usage():
    return "%s [vel sense]"%sys.argv[0]

if __name__ == "__main__":
    
    argv = rospy.myargv()

    initial_circle(0, 0)

    initial_square(0, 0)
   
    print "end"
