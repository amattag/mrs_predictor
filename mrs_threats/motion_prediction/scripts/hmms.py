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
# \file   hmms.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Hidden Markov Model module.
## In this module the main components and functions of a HMM are
## implemented.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

# Import required modules.
import numpy as np

# The Transition Probability Matrix A
def transProbMat(clustersN, stepsN):
	"""
	transProbMat: It computes a matrix with the transition probabilities
	of the states present in the system.
	
	Parameters:
	-----------
	clustersN: int
	  Number of clusters in the model.
	
	stepsN: int
	  Number of steps in the model. 
	
	Returns:
	--------
	A : array of floats.
	  Transition Probability Matrix.  
	"""
	A = np.zeros(( clustersN, stepsN, stepsN ))
	for m in xrange(clustersN):
		for t in xrange(stepsN):
			if t < stepsN - 1:
				# Normal node
				A[m, t, t + 1] = 1. / 3. # Advance
				A[m, t, t] = 2. / 3.     # Stay in node
			else:
				# End node
				A[m, t, t] = 1.
	return A

# State Prior
def statePrior(clustersN, stepsN):
	"""
	statePrior: It returns the state prior of the system.
	
	Parameters:
	-----------
	clustersN: int
	  Number of clusters in the model.
	
	stepsN: int
	  Number of steps in the model. 
	
	Returns:
	--------
	pi : array of floats.
	  Initial belief of the model.  
	"""
	# A motion always starts at the beginning of the trajectory.
	pi = np.zeros((clustersN, stepsN))
	for m in xrange(clustersN):
		pi[m, 0] = 1.0
	pi /= np.sum(pi)
	return pi


# The Filtering algorithm.
def filter(belief, A, means, invCov, obs, obsStep):
    """filter: To estimate the probability distribution over the state at the
       current time.
    
    Parameters
    ----------
    belief: array
      Initial belief (state prior).
    
    A: array of floats.
      Transition Probability Matrix.
      
    means: array
      Clusters array.
    
    invCov: array matrix
      Inverse of the Covariance Matrix
    
    obs: array
      Trajectory array used as an observation.
      
    obsStep: int
      State step of the coming observation.
    """
    clustersN, stepsN = belief.shape
    currentState = np.zeros((clustersN, stepsN))
    for m in xrange(clustersN):
        for currentStep in xrange(stepsN):
			diff = means[m, currentStep] - obs[obsStep]
			expo = -0.5 * np.dot(np.dot(diff, invCov), diff.transpose())
			for prevStep in xrange(stepsN):
				currentState[m, currentStep] += belief[m, prevStep] * A[m, prevStep, currentStep] * np.exp(expo) 
    currentState /= np.sum(currentState)
    belief[:] = currentState[:]
    
# The Prediction algorithm.
def predict(belief, A, stepsAhead):
    """predict: To estimate the probability distribution at different time steps
    in the future. 
    
    Parameters
    ----------
    belief: array
      State belief at time t.
    
    A: array
      Transition Probability Matrix.
      
    stepsAhead: int
      Number of states ahead in the future to make a prediction.
     
    Returns
    -------
    belief: array matrix
      Prediction of the State at a number of steps in the future.
    """
    clustersN, stepsN = belief.shape
    for t in xrange(stepsAhead):
        currentState = np.zeros((clustersN, stepsN))
        for m in xrange(clustersN):
            for currentStep in xrange(stepsN):
                for prevStep in xrange(stepsN):
                    currentState[m, currentStep] += A[m, prevStep, currentStep] * belief[m, prevStep]
        currentState /= np.sum(currentState)
        belief = currentState
    return belief
