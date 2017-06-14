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
# \file   clustering.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Clustering with the Expectation-Maximization (E-M) algorithm.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31
#
# Revision: $Id$: $file$ 2017-06-14

# Import required modules.
import rospy
import rospkg
import time
import sys
import random
import numpy as np
from os import path
import matplotlib.pyplot as plt

# import auxiliar modules.
import exp_max as em
import plotting as myplt
import probability as pobty

if __name__ == '__main__':
	try:
		# Init ros node.
		rospy.init_node('clustering',log_level=rospy.INFO)
		
		# Fetch incomming params.
		tjcsN = rospy.get_param('~tjcs_n')					# Number of Trajectories to load.
		clustersN = rospy.get_param('~clusters_n') 	# Number of Clusters.
		maxIter = rospy.get_param('~max_iter')   		# Maximum number of iterations for the E-M algoritm.
		tol = rospy.get_param('~tolerance')    			# error tolerance for the E-M algorithm.

		# Output input arguments.
		print "Number of Trajectories:", tjcsN
		print "Number of Clusters:", clustersN
		print "E-M iterations:", maxIter
		print "E-M Converge Threshold:", tol
		
		# Load trayectory data from the 'tjcs.npy' file.
		# A set of 1856 trajectories generated from the Edinburgh Informatics Forum 
		# Pedestrian Database: http://homepages.inf.ed.ac.uk/rbf/FORUMTRACKING/
		rospack = rospkg.RosPack()
		data = np.load(rospack.get_path('motion_prediction') + '/data/tjcs.npy','r')
		
		# Choose the first number of trajectories given by the tjcsN argument.
		tjcs = data[:tjcsN]
		
		# Plot selected trajectories.
		fig1 = plt.figure()
		myplt.plot_trajectories(tjcs, "Trajectory", "600x500+200+0")
		plt.pause(0.05)
		
		# Get the length of each trajectory.
		tjcsLength = tjcs.shape[1]
		
		# Select intial means for E-M algorithm.
		means = np.zeros((clustersN, tjcsLength, 2))
		meansIndex = random.sample(np.arange(tjcsN), clustersN)
		for m, n in zip( xrange(clustersN), meansIndex):
			means[m] = data[n]
			
		# Compute covariance Matrix.
		vtjcs=tjcs.reshape((tjcsN*tjcsLength, tjcs.shape[2]))
		covariance = np.round(np.cov(vtjcs.T)*np.eye(vtjcs.shape[1]))
		
		### Let's iterate with the E-M algorithm ###
		ll_old = 0
		i = 0
		while i < maxIter:
			# E-M algorithm.
			clusters = em.expectation(tjcs, means, covariance, pobty.t_gaussian)
			em.maximization(tjcs, clusters, means, pobty.t_zero)
			
			# Update log likelihoood.
			ll_new = 0.0
			for j in range(tjcsN):
				s = 0
				for k in range(clustersN):
					s += pobty.t_gaussian(means[k], covariance, tjcs[j])
				ll_new += np.log(s)
				
			# If the threshold is below the expected tol factor.
			if np.abs(ll_new - ll_old) < tol:
				# If found, replace the worst cluster with the worst represented
				# trajectory in order to improve the quality of the returned clusters.
				
				# Finds worst cluster and its index.
				clusterIndex, clusterScore = em.worst_cluster(clusters)
				
				# Finds the index of the worst represented trajectory.
				tjcIndex = em.worst_trajectory(clusters, clusterIndex, clusterScore, meansIndex, tjcs, covariance)
				
				# If the worst represented trajectory score is higher than the worst cluster score,
				# replace cluster with the found trajectory.
				if tjcIndex == -1:
					break
				else:
					print "Replacing cluster %i with trajectory %i" % (clusterIndex, tjcIndex)
					means[clusterIndex] = tjcs[tjcIndex]
					meansIndex.append(tjcIndex)
					ll_old = 0
					i = 0
					continue
			else:
				ll_old = ll_new
				i += 1
				
			print "Iteration Number: ", i
			
		# This is to plot the results of the E-M algorithm.
		fig2 = plt.figure()
		myplt.plot_clusters(clusters, tjcs, "Results after Clustering", "600x500+850+0")
		
		# Save means structre, which represents the clusters typical trayectories into
		# a npy file.
		timestr = time.strftime("%Y%m%d-%H%M%S")
		filename = path.join(rospack.get_path('motion_prediction') + '/data/'+timestr+"-clusters")
		#filename=path.join("data/"+timestr+"-clusters")
		np.save(filename, means)
		
		# Let's plot the found clusters.
		fig3 = plt.figure()
		myplt.plot_time_model(means, "Clusters", "600x500+200+600")
		
		# Plot cluster contribs
		cluster_contribs = np.sum(clusters, 0)
		fig4 = plt.figure()
		wm = plt.get_current_fig_manager()
		wm.window.wm_geometry("600x500+850+600")
		plt.title("Number of Trajectories per Cluster")
		plt.bar(np.arange(clustersN), cluster_contribs, color = myplt.generate_palette(clustersN))
		plt.show()
		
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
