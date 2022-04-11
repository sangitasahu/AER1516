import numpy as np
import ellipsoid as E
import time

def get_standard_form(plane):
    point = plane[0]
    normal = plane[1]
    d = point.dot(normal)
    standard_form = np.append(normal,d).tolist()
    return standard_form

def signed_dist_pt_plane(pt,V):
    n = V[1]
    p = V[0]
    distance = n.dot(pt-p)
    return distance

def pt_is_inside_poly(pt,Vs):
    check = True
    for V in Vs:
        if signed_dist_pt_plane(pt,V)>=0:
            check=False
    return check

def set_obs_Vs(obs,Vs):
    new_ob_set = []
    for pt in obs:
            if pt_is_inside_poly(pt,Vs):
                new_ob_set.append(pt)
    new_ob_set = np.array(new_ob_set)
    return new_ob_set  
    

def get_hyperplanes(obs_init,C,d):
    obs_remain = obs_init
    globalpoly = []
    while(obs_remain!=[]):
        hypplane = E.nearest_hyperplane(C,d,obs_remain)
        if not np.all(hypplane[1]==0):
            globalpoly.append(hypplane)
        obs_retain = []
        for pt in obs_remain:
            if pt_is_inside_poly(pt,globalpoly):
                
                obs_retain.append(pt)
        obs_remain = obs_retain
    return globalpoly  


def add_local_bbox(p1,p2,bbox):
    p1 = np.asarray(p1)
    p2 = np.asarray(p2)
    #P1 and P2 are 3*1 vectors
    #bbox is a l,b of the bbox around the line segment - 2D vector essentially
    
    #get unitvector parallel to the line segment:
    unitvec_parallel = (p2-p1)/np.linalg.norm(p2-p1)
    unitvec_norm = np.array([unitvec_parallel[1],-unitvec_parallel[0],0])
    unitvec_vert = np.array([0,0,0])
    unitvec_vert[0] = unitvec_parallel[1] * unitvec_norm[2] - unitvec_parallel[2] * unitvec_norm[1]
    unitvec_vert[1] = unitvec_parallel[2] * unitvec_norm[0] - unitvec_parallel[0] * unitvec_norm[2]
    unitvec_vert[2] = unitvec_parallel[0] * unitvec_norm[1] - unitvec_parallel[1] * unitvec_norm[0]
    hyperplanes = []
    #adding bbox planes parallel to line
    hyperplanes.append([p1+ unitvec_norm*bbox[1],unitvec_norm])
    hyperplanes.append([p1- unitvec_norm*bbox[1],-unitvec_norm])
    #adding bbox planes perpendicular to line
    hyperplanes.append([p2+ unitvec_parallel*bbox[0],unitvec_parallel])
    hyperplanes.append([p1- unitvec_parallel*bbox[0],-unitvec_parallel])   
    #adding bbox planes above and below the line
    hyperplanes.append([p1+unitvec_vert*bbox[2],unitvec_vert])
    hyperplanes.append([p1-unitvec_vert*bbox[2],-unitvec_vert])

    return hyperplanes

def obs_temp():
    obs = [[2.0299999713897705, 3.0799999237060547, 1.0299999713897705], [2.7799999713897705, 3.4800000190734863, 0.5199999809265137], [2.7799999713897705, 3.4800000190734863, 0.5799999833106995], [2.7300000190734863, 3.5299999713897705, 0.5199999809265137], [2.7799999713897705, 3.5299999713897705, 0.5199999809265137], [2.7300000190734863, 3.5299999713897705, 0.5799999833106995], [2.7799999713897705, 3.5299999713897705, 0.5799999833106995], [2.7799999713897705, 3.4800000190734863, 0.6299999952316284], [2.7799999713897705, 3.4800000190734863, 0.6800000071525574]]
    




    return obs