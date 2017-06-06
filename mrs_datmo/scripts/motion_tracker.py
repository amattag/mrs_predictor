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
from geometry_msgs.msg import Pose

# Pedsim-ros libraries
from pedsim_msgs.msg import TrackedPersons

def motion_tracking(TrackedPersons):
	"""
	simulate_people: It associates the items stored in the TrackedPersons list
	to its correspondent robot.
	
	Parameters:
	-----------
	TrackedPersons: Pedsim ROS message
	
	  A list with the people present in a pedsim_ros simulation.
	
	stepsN: int
	  Number of steps in the model.
	
	Returns:
	--------
	A : array of floats.
	  Transition Probability Matrix. 
	"""
	
	trackPublisher.publish(TrackedPersons)
	
	# For each tracked person...
	#for tracked_person in TrackedPersons.tracks:
		## Set the figure name.
		#subjectId = tracked_person.track_id
		#tracked_person.is_matched = True
		
	#rospy.Publisher('/pedsim/tracked_persons', TrackedPersons)
		 
if __name__ == '__main__':
	try:
		trackPublisher = rospy.Publisher('/summit1/tracked_persons', TrackedPersons, queue_size=10)
		# Init ros node.
		rospy.init_node('motion_tracking',log_level=rospy.INFO)
		rate = rospy.Rate(10)
		
		# Subscribe to PedSim tracked_persons list
		trackedPersonsTopic = "/pedsim/tracked_persons"
		rospy.Subscriber(trackedPersonsTopic, TrackedPersons, motion_tracking)
		rospy.loginfo("Subscribing to " + trackedPersonsTopic)
		
		rate.sleep()
	
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
