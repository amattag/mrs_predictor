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
# \file   patrol_fsm.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Patrolling with a Finite State Machine.
## A single state is implemented, that of driving to a particular 
## waypoint. Afterwards, all these states are chained together to get 
## the robot to patrol.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

import rospy
import numpy
import tf
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
            
if __name__ == '__main__':
	try:
		# Create a ROS node.
		rospy.init_node('patrol_fsm')
		
		# Fetch params.
		robot_name = rospy.get_param('~robot_name')
		robot_wps = rospy.get_param('~waypoints')
		
		# Construct Waypoint list.
		robot_wps = [[float(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in robot_wps.rstrip(']').split(']')]
		for i, point in enumerate(robot_wps):
			# Append state name.
			robot_wps[i].append(str(i+1))
		
		# Create the string "robot_name/move_base"
		move_base_robot = robot_name + "/move_base"
		
		# Create a SMACH State Machine.
		patrol = StateMachine(['succeeded', 'aborted', 'preempted'])
		
		# Construct Action Goals from the Waypoint list.
		with patrol:
			for i, w in enumerate(robot_wps):		
				# Define the Goal.
				goal_pose = MoveBaseGoal()
				goal_pose.target_pose.header.frame_id = 'map'
				goal_pose.target_pose.pose.position.x = w[0]
				goal_pose.target_pose.pose.position.y = w[1]
				goal_pose.target_pose.pose.position.z = 0.0
				
				# Convert Euler angle to quaternion.
				goal_radian = numpy.radians(w[2])
				q = tf.transformations.quaternion_from_euler(0, 0, goal_radian)
				q_msg = Quaternion(*q)
				
				goal_pose.target_pose.pose.orientation = q_msg
			
				## Add a Simple Action State to the State Machine.
				## It uses each one of the goals previously created. They return 
				## with GoalStatus.SUCCEEDED, causing this simple action state to 
				## return the outcome 'succeeded'
				StateMachine.add( str(i+1), 
				                  SimpleActionState(move_base_robot, MoveBaseAction, goal=goal_pose), 
												  transitions={'succeeded': robot_wps[(i + 1) % len(robot_wps)][3]}
												)
	
		#It sends each goal to the Nav Stack and waits for it to terminate.
		patrol.execute()
		
	except rospy.ROSInterruptException: pass
