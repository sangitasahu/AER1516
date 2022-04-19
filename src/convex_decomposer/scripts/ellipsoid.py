#!/usr/bin/env python2

import numpy as np
from tf.transformations import quaternion_matrix,quaternion_from_euler,quaternion_multiply
import time


def dist_pt_ellipsoid_centre(pt,Cinv,d):
  #t1 = time.time()
  distance = np.linalg.norm(Cinv.dot(pt-d))
  #distance = np.linalg.norm(np.linalg.pinv(C).dot(pt-d))
  return distance

def pt_is_inside_ep(pt,Cinv,d):
    if dist_pt_ellipsoid_centre(pt,Cinv,d)<=1:
      return True
    return False

def vec_to_rotation(v):
  rpy = np.array([0,np.arctan2(-v[2],np.linalg.norm(v[0:1])),np.arctan2(v[1],v[0])]) #RPY of vector V
  quaternion = quaternion_from_euler(0, np.arctan2(-v[2],np.linalg.norm(v[0:1])),np.arctan2(v[1],v[0]))
  R = quaternion_matrix(quaternion)[0:3,0:3]
  return R

def closest_pt_ep(obs,Cinv,d):
    dist_list = [dist_pt_ellipsoid_centre(pt,Cinv,d) for pt in obs]
    min_idx = dist_list.index(min(dist_list))
    p = obs[min_idx]
    return p

def nearest_hyperplane(Cinv,d,obs):
  pw = closest_pt_ep(obs,Cinv,d)
  n = Cinv.dot(Cinv.transpose().dot(pw-d))
  norm_n = np.linalg.norm(n)
  if norm_n ==0:
    n_normalized = n
  else:
    n_normalized = n/norm_n
  closest_hp = [pw,n_normalized]
  return closest_hp


def find_ellipsoid(p1,p2,offset_x,obs,inflate_distance):
    p1 = p1.astype(np.double)
    p2 = p2.astype(np.double)
    eps_limit = 1e-8
    d = np.divide((p1+p2),2)

    C =  np.linalg.norm(p1-p2)/2 * np.identity(3).astype(np.double)  
    axes = np.linalg.norm(p1-p2)/2 + np.array([0,0,0]).astype(np.double)
#    print(p1 ,"to", p1)
    C[0,0] = C[0,0]+offset_x
    axes[0] = axes[0]+offset_x
    if axes[0]>0:
      ratio = axes[1]/axes[2]
      C = C.dot(ratio)
      axes = axes.dot(ratio)
    R = vec_to_rotation(p2 - p1)
 #   print("R ", R)
    #Form the sphere and transform it to be around the segment
    C = R.dot(C.dot(R.transpose()))
    d = np.divide((p1+p2),2)
    # Now we squish this sphere to touch the obstacles 
    Rf = R
    #Inflate obstacle  by inflation distance first.
    obs_inflated = []
    for pt in obs:
        p = R.transpose().dot(pt-d)
        pt_inflated = R.dot(np.array([p[0]-np.sign(p[0]) * inflate_distance,p[1]-np.sign(p[1]) * inflate_distance,p[2]-np.sign(p[2]) * inflate_distance]))+d
        obs_inflated.append(pt_inflated)
  
    obs_inside = []
    Cinv = np.linalg.pinv(C)
    for pt in obs_inflated:
      if pt_is_inside_ep(pt,Cinv,d):
        obs_inside.append(pt)

    #return C,d,obs_inflated
    obs_inside_ = obs_inside
    obs = np.asarray(obs_inflated).astype(np.double) # This obs is necessary to form our polyhedrons!
    #work on ellipsoid squishing only if obstacles that are inflated are inside the current sphere
    #So we work on the obs_inside list
    while (obs_inside!=[]):
        #Cinv = np.linalg.pinv(C)
        pw = closest_pt_ep(obs_inside,Cinv,d)
        #print("pt in question:", pw)
        p=R.transpose().dot(pw-d)
        roll = np.arctan2(p[2],p[1])
        Rf_ = quaternion_matrix([0,np.cos(roll/2),np.sin(roll/2),0])[0:3,0:3]
        Rf = R.dot(Rf_)
        #print("Rf ", Rf)
        p = Rf.transpose().dot(pw-d)
        if (p[0]<axes[0]):
            axes[1] = np.abs(p[1]/np.sqrt(1-(p[0]/axes[0])**2))
        #form new ellipsoid
        new_C = np.diag([axes[0],axes[1],axes[1]]).astype(np.double)
        #transform the new ellipsoid around segment 
        C = Rf.dot(new_C.dot(Rf.transpose()))
        obs_retain = []
        #check if the new ellipsoid still has obs inside, if yes repeat.
        Cinv = np.linalg.pinv(C)
        for pt in obs_inside:
            if (1-dist_pt_ellipsoid_centre(pt,Cinv,d)>eps_limit):                
                obs_retain.append(pt)
        obs_inside = obs_retain

    #Done with loop for y axis,  proceed to z axis
    C = np.diag(axes)
    C = Rf.dot(C.dot(Rf.transpose()))
    Cinv = np.linalg.pinv(C)
    obs_inside_new = []
    for pt in obs_inside_:
      if pt_is_inside_ep(pt,Cinv,d):
        obs_inside_new.append(pt)
    obs_inside = obs_inside_new    
    while (obs_inside !=[]):
        #Cinv = np.linalg.pinv(C)
        pw = closest_pt_ep(obs_inside,Cinv,d)
        p=Rf.transpose().dot(pw-d)
        if (1-(p[0]/axes[0])**2 - (p[1]/axes[1])**2 > eps_limit):
            axes[2] = np.abs(p[2]/np.sqrt(1-(p[0]/axes[0])**2 - (p[1]/axes[1])**2))
        new_C = np.diag(axes).astype(np.double)
        C = Rf.dot(new_C.dot(Rf.transpose()))
        obs_retain = []
        Cinv = np.linalg.inv(C)
        for pt in obs_inside:
            if (1-dist_pt_ellipsoid_centre(pt,Cinv,d)>eps_limit):
                obs_retain.append(pt)
        obs_inside = obs_retain
    return C,d,obs#use the last return -> a list that can bve returned to display obstacles on xnewcloud publisher, for debug only