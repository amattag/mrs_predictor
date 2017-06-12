#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Universidad Politecnica de Madrid.
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
# \file   monitor.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## It creates the Monitor node that will listen for the ready messages 
## comming from each robot and announces when all are ready. 
#
# Version: 1.0.0 $Id$: $file$ 2017-06-10

import rospy
import sys

#import local msgs
from mrs_navigation.msg import RobotStatus

def monitor_callback(msgRobotStatus):
	"""
	monitor_callback: It checks if all the robots are ready.
	
	Parameters: 
	-----------
	status_msg : ROS RobotStatus message.
	
	Returns:
	--------
	status_msg : ROS RobotStatus message.
	  A ROS msg with the current monitor status.
	"""
	global robots_count
	global print_command
	# Check if all robots are ready.
	if (robots_count != team_size):
		rospy.loginfo("Robot %s is ready!\n", msgRobotStatus.robot_id)
		robots_ready = True
		robots_count += 1
	elif (robots_count == team_size):
		if not print_command:
			rospy.loginfo("All robots GO!")
			print_command = True
		status_msg = RobotStatus()
		status_msg.header.stamp = rospy.Time.now()
		status_msg.header.frame_id = "monitor"
		team_status_pub.publish(status_msg)
				          
if __name__ == '__main__':
	try:
		# Init ROS node.
		rospy.init_node('mrs_monitor',log_level=rospy.INFO)
		
		robots_count = 0
		robots_ready = False
		print_command = False
		
		# Fetch incomming params.
		team_size = rospy.get_param('~team_size')
		
		rospy.sleep(4)
									
		# Publish and subscribe to team status messages.
		team_status_pub = rospy.Publisher('/mrs_predictor/team_status', RobotStatus, queue_size=10)
		team_status_sub = rospy.Subscriber('/mrs_predictor/team_status', RobotStatus, monitor_callback)
		
		rospy.loginfo("Waiting for robots to connect...")
		
		# Keeps python from exiting until this node is stopped.
		rospy.spin()
		
	except rospy.ROSInterruptException: pass
