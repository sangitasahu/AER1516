#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Local Planner
# Bezier Curve helper functions
#

from __future__ import division, print_function, absolute_import

import numpy as np
from scipy.special import comb

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of degree n, i as a function of t
    """

    return comb(n, i) * ( t**i ) * (1 - t)**(n-i)

def bezier_curve(P,dT,n=3,num_disc=100):
    """
    Calculate bezier curve values for multiple axes and time intervals

    Parameters
    ----------
    P : Control points. Array of shape (axes x points x time intervals)
    dT : Time intervals for each curve segment

    Returns
    -------
    t - Time vector for plotting. Shape (time)
    x - Position vector as a function of time for plotting. Shape (axes x time)

    """
    
    n_int = dT.shape[0]
    t_norm = np.arange(0,1+1/num_disc,1/num_disc)
    
    # Build Berstein Polynomials
    b_poly = np.zeros((n+1,t_norm.shape[0]))
    for i in range(n+1):
        b_poly[i,:] = bernstein_poly(i,n,t_norm)
    
    x_temp = np.zeros((P.shape[0],t_norm.shape[0],n_int))
    
    for i in range(n_int):
        x_temp[:,:,i] = np.matmul(P[:,:,i],b_poly)
    
    t_temp = t_norm*dT[0]
    
    for i in range(1,dT.shape[0]):
        t_temp = np.hstack((t_temp,t_temp[-1]+t_norm*dT[i]))
    
    x = x_temp.reshape((x_temp.shape[0],x_temp.shape[1]*x_temp.shape[2]),order='F')
    t = t_temp
    
    return x,t

def bezier_interpolate(P,dT,t,n=3):
    """
    Interpolate value for a piecewise Bezier curve in multiple axes

    Parameters
    ----------
    P : Control points. Array of shape (axes x points x time intervals)
    dT : Time intervals for each curve segment
    t : Time to interpolate at
    n: Degree

    Returns
    -------
    x - Interpolated value. Shape (axes,)
    """

    # Clamp t to live between 0 and max time
    t_cum = np.cumsum(dT)
    t_clamp = np.clip(t,0,t_cum[-1])
    ind_curr = np.min(np.argwhere(t_clamp<=t_cum))

    t_norm = (t_clamp-t_cum[ind_curr]+dT[ind_curr])/dT[ind_curr]

    # Build Berstein polynomial basis function values at interpolated point
    b_poly = np.zeros(n+1)
    for i in range(n+1):
        b_poly[i] = bernstein_poly(i,n,t_norm)

    # Result is weighted sum
    return np.matmul(P[:,:,ind_curr],b_poly.reshape((n+1,1)))