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
# \file   ploting.py
# \author Antonio Matta <antonio.matta@upm.es>
#
## Plotting module.
## A module with auxiliary functions to plot clusters and trajectories.
#
# Version: 1.0.0 $Id$: $file$ 2017-05-31

# Import required modules.
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

palettes = {}

def generate_palette(count):
  """Generates a color id to every cluster.
  
  Parameters
  ----------
  count: int
    Number of clusters or trajectories.
  
  Returns
  -------
  palettes: array_like
    An array of colored ids. One for each cluster.
  """  
  if count in palettes:
    return palettes[count]
  samples = np.random.randint( 0, 255, ( 1000, 3 ) ) 
  km = KMeans( count, init = "k-means++" )
  km.fit( samples )
  palettes[count] = km.cluster_centers_ / 255.0
  return palettes[count]


def plot_trajectories(tjcs, title, dims):
  """Plots a set of trajectories.
  
  Parameters
  ----------
  tjcs: array 
    A set of 2D (x, y) trajectories. Each point of is a float number.
    
  title: string
    A string with the title of the plot.
  
  dims: string
    A string with the dimensions of the window of the plot.
  """ 
  wm = plt.get_current_fig_manager()
  wm.window.wm_geometry(dims)
  plt.title(title)
  plt.axis([0,16,0,12])
  plt.xlabel('X (mts)')
  plt.ylabel('Y (mts)')
  
  if len(tjcs) < 50:
    palette = generate_palette(len(tjcs))
    for n, t in enumerate(tjcs):
      t = np.array(t)
      plt.plot( t[:,0], t[:,1], "-", c = palette[n] )
      plt.plot( t[-1,0], t[-1,1], "o", markersize = 5.0, c = palette[n] )
  else:
    for t in tjcs:
      t = np.array(t)
      plt.plot(t[:,0], t[:,1], "-")


def plot_clusters(clusters, tjcs, title, dims):
  """Plots all the clusters and their corresponding trajectories.
  
  Parameters
  ----------
  clusters: array_like
    A set of clusters. Each point of is a float number.
  
  tjcs: array_like
    A set of 2D (x, y) trajectories. Each point of is a float number.
    
  title: string
    A string with the title of the plot.
  
  dims: string
    A string with the dimensions of the window of the plot.
  """ 
  wm = plt.get_current_fig_manager()
  wm.window.wm_geometry(dims)
  plt.axis([0,16,0,12])
  plt.xlabel('X (mts)')
  plt.ylabel('Y (mts)')
  plt.title(title)

  clusterLenght = clusters.shape[1]
  palette = generate_palette(clusterLenght)
  clustersTot = np.sum(clusters)
  for n, traj in enumerate(tjcs):
    traj = np.array(traj)
    c = np.array( [0., 0., 0.] )
    if np.sum(clusters[n, :]) / clustersTot > 1E-4:
      clusterTot = np.sum(clusters[n, :])
      for m in xrange(clusterLenght):
        c += palette[m] * clusters[n, m]/clusterTot
      plt.plot(traj[:,0], traj[:,1], "-", color = c)
    else:
      c = np.array([0.8, 0.8, 0.8])
      plt.plot(traj[:,0], traj[:,1], "-", color = c, alpha = 0.3)
        

def plot_time_model(tjcs, title, dims):
  """From every cluster, plot a typical trajectory that represents the 
     entire cluster.
  
  Parameters
  ----------
  tjcs: array
      An array containing all the clusters.
      
  title: string
    A string with the title of the plot.
  
  dims: string
    A string with the dimensions of the window of the plot.
  """ 
  wm = plt.get_current_fig_manager()
  wm.window.wm_geometry(dims)
  plt.title(title)
  plt.grid()
  plt.xticks(np.arange(0, 16, 1.0))
  plt.yticks(np.arange(0, 12, 1.0))
  plt.xlabel('X (mts)')
  plt.ylabel('Y (mts)')  
  
  palette = generate_palette(len(tjcs))
  for tjcsN, tjc in enumerate(tjcs):
    tjc = np.array(tjc)
    plt.plot(tjc[:,0], tjc[:,1], "o", c = palette[tjcsN])


def plot_prediction( clusters, obs, obsStep, obsIni, belief, prediction ):
  """ Plot the observations, current belief state and predicted belief state at
  a time step given by the user.
  
  Parameters
  ----------
  clusters: array.
    An array containing all the clusters.
      
  obs: array.
    A trajectory used as an observation.
    
  obsStep: int
    step of the prediction.
    
  obsIni: int 
    Id of the observation being predicted.
       
  belief: matrix array
    Belief state at time t.
    
  prediction: matrix array
    Belief state at time t+steps.
  """ 
  wm = plt.get_current_fig_manager()
  wm.window.wm_geometry("600x500+200+600")
  plt.grid()
  plt.xticks(np.arange(0, 16, 1.0))
  plt.yticks(np.arange(0, 12, 1.0))
  plt.xlabel('X (mts)')
  plt.ylabel('Y (mts)')  
  plt.title("Prediction for Obs %s. State Step: " %obsIni )
  
  # Plot the observations (trajectories)
  plt.plot(obs[:,0], obs[:,1], "x", c = (0.8, 0.1, 0.3))
  
  for n, t in enumerate(clusters):
    t = np.array(t)
    for i in xrange(len(t)):
      # Plot the belief state at the current time step.  
      plt.plot(t[i,0], t[i,1], "o", markersize = 15. * belief[n, i], c = (0.2, 0.6, 0.2))
      
      # Plot the belief state in a future time step. 
      plt.plot(t[i,0], t[i,1], "D", markersize = 60. * prediction[n, i], c = (1.0, 0.3, 0.3))
