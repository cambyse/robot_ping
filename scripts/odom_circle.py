#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

from robot_ping.srv import OdomCircleCommand

class OdomCircle():
    def turn(self, val):
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Initialize the position variable as a Point type
        position = Point()
            
        # Loop once for each leg of the trip
        #############
        # Initialize the movement command
        move_cmd = Twist()
            
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed
        
        # Get the starting position values     
        (position, rotation) = self.get_odom()
                        
        x_start = position.x
        y_start = position.y

        # Track the last angle measured
        last_angle = rotation

        # Track how far we have turned
        turn_angle = 0

        while abs(turn_angle + self.angular_tolerance) < abs(self.goal_angle) and not rospy.is_shutdown():
            
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            r.sleep()
                
            # Get the current rotation
            (position, rotation) = self.get_odom()
                
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)
                
            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
            
        # Stop the robot for good
        self.cmd_vel.publish(Twist())

        return True
    
    
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_circle_server', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
       
        
        # Set the forward linear speed to 0.2 meters per second 
        self.linear_speed = 0.0
        
        # Set the travel distance in meters
        self.goal_distance = 0.0

        # Set the rotation speed in radians per second
        self.angular_speed = 0.25
        
        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = radians(2.5)
        
        # Set the rotation angle to Pi radians (180 degrees)
        self.goal_angle = 2*pi

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
               
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  

        print "creating service..."
        self.service = rospy.Service('odom_circle', OdomCircleCommand, self.turn)
        print "Ready to turn..."
        rospy.spin()

        while not rospy.is_shutdown():
            rospy.sleep(1.0)
      
        ############
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    #try:
        OdomCircle()
    #except:
     #   rospy.loginfo("initial circle node terminated.")

