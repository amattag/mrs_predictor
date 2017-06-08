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
# \file   motion_tracker.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Associates a tracking list to its correspondent robot.
#
# Version: 1.0.0 $Id$: $file$ 2017-06-05

import rospy
import math
from sensor_msgs.msg	import	LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

# Pedsim-ros libraries
from pedsim_msgs.msg import TrackedPersons, TrackedPerson

def motion_tracking(msg):
	"""
	motion_tracking: It associates the items stored in the TrackedPersons list
	to its correspondent robot.
	
	Parameters:
	-----------
	TrackedPersons: Pedsim ROS message
	  A list with the people present in a pedsim_ros simulation.
	
	Returns:
	--------
	TrackedPersons : a TrackedPersons msg
	  a message that contains a list of tracked persons.
	"""
	global seqCounter
	robotTrackedPersons = TrackedPersons()
	robotTrackedPersons.header.seq = seqCounter
	robotTrackedPersons.header.frame_id = "odom"
	robotTrackedPersons.header.stamp = rospy.Time.now()
	
	# For each tracked person...
	for tracked_person in msg.tracks:
		# Find position between TrackedPerson and robot.
		distance = math.sqrt( math.pow((g_robot_pose.position.x - tracked_person.pose.pose.position.x), 2) \
		                     + math.pow((g_robot_pose.position.y - tracked_person.pose.pose.position.y),2) )
		
		if distance < g_range_ahead:
			tracked_person.is_matched = True
			trackedPerson = TrackedPerson()
			trackedPerson.track_id = tracked_person.track_id
			trackedPerson.is_occluded = tracked_person.is_occluded
			trackedPerson.is_matched = True
			trackedPerson.detection_id = tracked_person.detection_id
			trackedPerson.age = tracked_person.age
			trackedPerson.pose = tracked_person.pose
			trackedPerson.twist = tracked_person.twist
			robotTrackedPersons.tracks.append(trackedPerson)
					
	trackPublisher.publish(robotTrackedPersons)
	seqCounter += 1
					
def laser_callback(Scan):
	"""
	laser_callback: It reads a laser scan and computes its range.
	
	Parameters:
	-----------
	Scan: LaserScan message
	
	  A ROS LaserScan message.
	
	Returns:
	--------
	g_range_ahead : float
	  The Laser range.
	"""
	global g_range_ahead
	g_range_ahead = Scan.ranges[len(Scan.ranges)/2]
	
def odometry_callback(msg):
	"""
	odometry_callback: It Gets the current pose of the robot.
	
	Parameters:
	-----------
	msg: ROS Odometry message
	  Odometry of the robot.
	
	Returns:
	--------
	g_robot_pose : a ROS Pose msg.
	  a message that contains the current pose of the robot.
	"""
	global g_robot_pose
	g_robot_pose = msg.pose.pose
	
		 
if __name__ == '__main__':
	try:
		
		# Robot TrackedPersons msg to be published.
		trackPublisher = rospy.Publisher('/summit1/tracked_persons', TrackedPersons, queue_size=10)
		
		# Init ROS node.
		rospy.init_node('motion_tracking',log_level=rospy.INFO)
		
		# Subscribe to the robot's laser.
		g_range_ahead	=	0.0	
		scan_sub	=	rospy.Subscriber('/summit1/base_scan', LaserScan, laser_callback)
		
		# Subscribe to the robot's Odometry.
		g_robot_pose = Pose()
		rospy.Subscriber('/summit1/odom', Odometry, odometry_callback)
		
		# Subscribe to PedSim TrackedPersons tracks list.
		seqCounter = 0
		trackedPersonsTopic = "/pedsim/tracked_persons"
		rospy.Subscriber(trackedPersonsTopic, TrackedPersons, motion_tracking)
		rospy.loginfo("Subscribing to " + trackedPersonsTopic)
		
		# Keeps python from exiting until this node is stopped.
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
