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
# \file   motion_pred_tests.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## A set of unitary tests for the motion prediction module.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

from nose.tools import *
import numpy as np
import random
import motionPred
from motionPred import probability
from motionPred import expMax

def setup():
	"""This method is run once before _each_ test method is executed"""
	global tjcsN       # Number of Trajectories.
	global tjcsLength  # Lenght of each Trajectory.
	global data        # Structure to store Trajectory data.
	global clustersN   # Number of Clusters.
	
	tjcsN = 200
	tjcsLength = 76
	clustersN = 13 
	data = np.load("motionPred/data/tjcs.npy")
	
	print "SETUP Check!"
	
def teardown():
	"""This method is run once after _each_ test method is executed"""
	print "TEAR DOWN!"
	
def test_trajectory():
	global tjcs
	tjcs = data[:tjcsN]
	assert_equal(tjcs.shape[0], 200)
	assert_equal(tjcs.shape[1], 76)
	print "Trajectory Structure OK!"
	
def test_means():
	global means
	means = np.zeros((clustersN, tjcsLength, 2))
	meansIndex = random.sample(np.arange(tjcsN), clustersN)
	for m, n in zip( xrange(clustersN), meansIndex):
		means[m] = data[n]
		
	assert_equal(means.shape[0], 13)
	assert_equal(means.shape[1], 76)
	assert_not_equal(np.sum(means), 0)
	print "Cluster Structure OK!"
	
def test_covariance():
	global covariance
	vtjcs=tjcs.reshape((tjcsN*tjcsLength, tjcs.shape[2]))
	covariance = np.round(np.cov(vtjcs.T)*np.eye(vtjcs.shape[1]))
	assert_equal(covariance.shape[0], covariance.shape[1]) # covariance matrix must be square
	assert_equal(covariance.shape[0], tjcs.shape[2]) # covariance matrix and trajectory matrix, same dimension.
	print "Covariance matrix OK!"
	
def test_zero():
	probability.t_zero(tjcs[20])
	assert_equal(np.sum(tjcs[20]), 0)
	print "Zero method OK!"
	
def test_gaussian():
	gauss = probability.t_gaussian(means[0], covariance, tjcs[10])
	assert_equal(gauss.ndim, 0)
	print "Gaussian method OK!"
	
def test_expectation():
	global clusters
	clusters = expMax.expectation(tjcs, means, covariance, probability.t_gaussian)
	assert_equal(clusters.shape[0], tjcs.shape[0])
	assert_equal(clusters.shape[1], means.shape[0])
	assert_not_equal(np.sum(clusters), 0)  
	print "Expectation method OK!"
	
def test_maximization():
	expMax.maximization(tjcs, clusters, means, probability.t_zero)
	assert_equal(means.shape[0], 13)
	assert_equal(means.shape[1], 76)
	assert_not_equal(np.sum(means), 0)
	print "Maximization method OK!"
	
