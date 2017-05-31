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
# \file   probability.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Probability module.
## A module with typical probability functions used in robotics for 
## clustering techniques.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

# Import required modules.
import numpy as np
from scipy import linalg

# Auxiliar functions: t_gaussian, t_zero and t_cummulate functions.
def t_zero(trajectory):
    """ Set the values of a trajectory to zero.
    
    Parameters:
    ----------
    trajectory: array
      A set of 2D (x, y) points. Each point of is a float number.
    """
    trajectory *= 0.0


def t_cummulate(t1, weight, t2):
    """ Cummulate weighted tjc t2 into t1
    
    Parameters:
    -----------
    weight: array
      Array with weight values.
      
    t1: array
      Array to be weighted.
    
    t2: array
      Array use to weight another array.
    """
    t1 += weight * t2
  
        
def t_gaussian(mean, covariance, value):
    """ Multivariate Gaussian Probability Distribution Function (PDF)
    
    Parameters:
      mean: array
        Array with  the trajectories used as means.
         
      covariance: matrix array
        Diagonal matrix with the covariance data.
        
      value: array
        Trajectories data.
    
    Return:
      exp: array
        An array of expected clusters. 
    """
    invCov = linalg.inv(covariance)
    diff = mean - value
    dist = -0.5 * np.dot(np.dot(diff, invCov), diff.transpose())
    exp = np.exp(np.diagonal(dist))
    return np.multiply.reduce(exp)
