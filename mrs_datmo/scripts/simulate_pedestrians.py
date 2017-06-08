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
# \file   simple_pedestrians.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Simulate pedestrians of a pedsim_ros simulation into a Gazebo world
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

# Pedsim-ros libraries
from pedsim_msgs.msg import TrackedPersons

def simulate_people(TrackedPersons):
	"""
	simulate_people: It grabs the items stored in the TrackedPersons list
	and sends them to Gazebo.
	
	Parameters:
	-----------
	TrackedPersons: Pedsim ROS message
	
	  A list with the people present in a pedsim_ros simulation. 
	"""
	# Initial pose of the cylinder
	initial_pose = Pose()
	
	# For each tracked person...
	for tracked_person in TrackedPersons.tracks:
		# Set the figure name.
		subjectId = tracked_person.track_id
		figure_name = "cylinder_%d" % subjectId
		
		# Delete model if already exists.
		#delete_model_prox(figure_name)
		
		# Setting the initial pose.
		initial_pose.position.x = tracked_person.pose.pose.position.x
		initial_pose.position.y = tracked_person.pose.pose.position.y
		initial_pose.position.z = tracked_person.pose.pose.position.z
		
		initial_pose.orientation.x = tracked_person.pose.pose.orientation.x
		initial_pose.orientation.y = tracked_person.pose.pose.orientation.y
		initial_pose.orientation.z = tracked_person.pose.pose.orientation.z
		initial_pose.orientation.w = tracked_person.pose.pose.orientation.w
		
		# Spawn model.
		spawn_model_prox(figure_name, sdff, "", initial_pose, "world")
		
		# Model state
		ms = ModelState()
		ms_subjectId = tracked_person.track_id
		ms_figure_name = "cylinder_%d" % ms_subjectId
		ms.model_name = ms_figure_name
		ms.pose.position.x = tracked_person.pose.pose.position.x
		ms.pose.position.y = tracked_person.pose.pose.position.y
		ms.pose.position.z = tracked_person.pose.pose.position.z
		
		ms.pose.orientation.x = tracked_person.pose.pose.orientation.x
		ms.pose.orientation.y = tracked_person.pose.pose.orientation.y
		ms.pose.orientation.z = tracked_person.pose.pose.orientation.z
		ms.pose.orientation.w = tracked_person.pose.pose.orientation.w
		
		ms.reference_frame = 'world'
  
		# Set new model state
		set_ms = SetModelState()
		set_ms.model_state = ms
		client_ms_prox.call(ms)
		 
if __name__ == '__main__':
	try:
		# Init ros node.
		rospy.init_node('simulate_people',log_level=rospy.INFO)
		
		# Open the sdf file.
		# Get the file path of mrs_datmo package.
		rospack = rospkg.RosPack()
		f = open(rospack.get_path('mrs_datmo') + '/models/cylinder.sdf','r')
		sdff = f.read()
		
		rospy.wait_for_service("gazebo/delete_model")
		rospy.wait_for_service('gazebo/spawn_sdf_model')
		rospy.wait_for_service("gazebo/set_model_state")
		spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
		delete_model_prox = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
		client_ms_prox = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		
		trackedPersonsTopic = "/pedsim/tracked_persons"
		rospy.Subscriber(trackedPersonsTopic, TrackedPersons, simulate_people)
		rospy.loginfo("Subscribing to " + trackedPersonsTopic)
	
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
