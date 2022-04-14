#!/usr/bin/env python2

import numpy as np
from scipy.spatial.transform import Rotation
import time

def dist_pt_ellipsoid_centre(pt,C,d):
  distance = np.linalg.norm(np.linalg.pinv(C).dot(pt-d))
  return distance

def pt_is_inside_ep(pt,C,d):
    if dist_pt_ellipsoid_centre(pt,C,d)<=1:
      return True
    else:
      return False

def vec_to_rotation(v):
  rpy = np.array([0,np.arctan2(-v[2],np.linalg.norm(v[0:1])),np.arctan2(v[1],v[0])]) #RPY of vector V
  rot = Rotation.from_euler('xyz', rpy, degrees=False)
  R = rot.as_dcm()
  return R

def closest_pt_ep(obs,C,d):
    dist_list = [dist_pt_ellipsoid_centre(pt,C,d) for pt in obs]
    min_idx = dist_list.index(min(dist_list))
    p = obs[min_idx]
    return p

def set_obs_ep(obs,C,d):
    new_ob_set = []
    for pt in obs:
            if pt_is_inside_ep(pt,C,d):
                new_ob_set.append(pt)
    new_ob_set = np.array(new_ob_set)
    return new_ob_set  

def nearest_hyperplane(C,d,obs):
  pw = closest_pt_ep(obs,C,d)
  Cinv = np.linalg.inv(C)
  n = Cinv.dot(Cinv.transpose().dot(pw-d))
  norm_n = np.linalg.norm(n)
  if norm_n ==0:
    n_normalized = n
  else:
    n_normalized = n/norm_n
  closest_hp = [pw,n_normalized]
  return closest_hp


def find_ellipsoid(p1,p2,offset_x,obs,inflate_distance):
    xobs_ = obs
    eps_limit = 1e-10
    p1 = np.asarray(p1).astype(np.float)
    p2 = np.asarray(p2).astype(np.float)
    d = np.divide((p1+p2),2)
    C =  np.linalg.norm(p1-p2)/2 * np.identity(3)  
    C[0,0] +=   offset_x
    axes = np.linalg.norm(p1-p2)/2 + np.array([offset_x,0,0])
    
    if axes[0]>0:
      ratio = axes[1]/axes[2]
      C = C.dot(ratio)
      axes = axes.dot(ratio)

    R = vec_to_rotation(p2 - p1)
    C = R.dot(C.dot(R.transpose()))
    d = np.divide((p1+p2),2)

    Rf = R
    obs_inflated = []
    obs_preserve = obs
    

    for pt in obs:
        p = R.transpose().dot(pt-d)
        obs_check = R.dot(np.array([p[0]-np.sign(p[0]) * inflate_distance,p[1]-np.sign(p[1]) * inflate_distance,p[2]-np.sign(p[2]) * inflate_distance]))+d
        if pt_is_inside_ep(obs_check,C,d):
            obs_inflated.append(obs_check)
    obs = np.asarray(obs_inflated)
    obs_inside = obs
    while (obs_inside!=[]):
        pw = closest_pt_ep(obs_inside,C,d)
        p=R.transpose().dot(pw-d)
        roll = np.arctan2(p[2],p[1])
        quat_roll = Rotation.from_quat([np.cos(roll/2),np.sin(roll/2),0,0])
        Rf = R.dot(quat_roll.as_dcm())
        p = Rf.transpose().dot(pw-d)
        if (p[0]<axes[0]):
            axes[1] = np.abs(p[1]/np.sqrt(1-(p[0]/axes[0])**2))
        new_C = np.diag([axes[0],axes[1],axes[1]])
        C = Rf.dot(new_C.dot(Rf.transpose()))
        obs_retain = []
        for pt in obs_inside:
            if (1-dist_pt_ellipsoid_centre(pt,C,d)>eps_limit):                
                obs_retain.append(pt)
        obs_inside = obs_retain
    #Done with loop
    C = np.diag(axes)
    C = Rf.dot(C.dot(Rf.transpose()))

    obs_inside = set_obs_ep(obs,C,d)
    while (obs_inside !=[]):
        pw = closest_pt_ep(obs_inside,C,d)
        p=Rf.transpose().dot(pw-d)
        if (1-(p[0]/axes[0])**2 - (p[1]/axes[1])**2 > eps_limit):
            axes[2] = np.abs(p[2]/np.sqrt(1-(p[0]/axes[0])**2 - (p[1]/axes[1])**2))
        new_C = np.diag(axes)
        C = Rf.dot(new_C.dot(Rf.transpose()))
        obs_retain = []
        for pt in obs_inside:
            if (1-dist_pt_ellipsoid_centre(pt,C,d)>eps_limit):
                obs_retain.append(pt)
        obs_inside = obs_retain

    return C,d,[] #use the last return -> a list that can bve returned to display obstacles on xnewcloud publisher, for debug only