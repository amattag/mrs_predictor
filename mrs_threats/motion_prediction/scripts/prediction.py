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
# \file   hmms_pred.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Human motion prediction using a HMM
## Demonstration of how to predict human motion using a Hidden Markov Model.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

# Import required modules.
import numpy as np
import matplotlib.pyplot as plt
import random

# import auxiliar modules.
import plotting as myplt
import hmms as myHMM

# Load clusters trayectory data from the 'clusters.npy' file.
clusters = np.load("data/clusters.npy")

# Let's plot those clusters again.
fig1 = plt.figure()
myplt.plot_time_model(clusters, "Clusters", "600x500+200+0")

# Resample the original clusters every two samples (0.6 seconds).
clustersN, origSteps, dim = clusters.shape
stepsN = int(np.ceil(origSteps/3.0))
means = np.zeros((clustersN, stepsN, dim))
for m in xrange(clustersN):
    for t in xrange(stepsN):
        count = 0.0
        for new_t in xrange(3 * t, min(3 *(t + 1), origSteps)):
            means[m, t] += clusters[m, new_t]
            count += 1.0
        means[m, t] /= count

# Plot the new clusters
fig2 = plt.figure()
myplt.plot_time_model(means, "Clusters", "600x500+850+0")

### Let us create the HMM model. ###

# First step: The Transition Probability Matrix A
A = myHMM.transProbMat(clustersN, stepsN)

# Second step: The state prior.
pi = myHMM.statePrior(clustersN, stepsN)

# Third step: The global covariance matrix for the observation probabilities.
covariance = np.array( [[4.0, 0.0], [0.0, 4.0]] )
invCov= np.linalg.inv(covariance)

### Using the HMM model for prediction. ###    
### Test the filtering and prediction algorithms with some test data. ###

# Grab the last 100 trajectories as test data.
data = np.load("data/tjcs.npy")
test_data = data[-100:]

iniState = 0    # initial State
interval = 2    # State Step interval
stepsAhead = 10 # Number of steps ahead in the future
endState = stepsN   # final State

# Make predictions using a set of observations from the test data.
obsIni = 34  # Initial observation
obsFin = 37  # Final observation
for obs in test_data[obsIni:obsFin]:   
    belief = pi.copy()
    for obsStep in xrange(iniState, (endState+1), interval):
        myHMM.filter(belief, A, means, invCov, obs, obsStep)
        prediction = myHMM.predict(belief, A, stepsAhead)       
        # Plot prediction
        fig3 = plt.figure()
        myplt.plot_prediction(means, obs[:obsStep], obsStep, obsIni, belief, prediction)
        #plt.pause(0.05)
        plt.show()
    obsIni +=1
