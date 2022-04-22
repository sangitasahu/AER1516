import numpy as np
import ellipsoid as E
from numpy.linalg import norm

def get_standard_form(plane):
    point = plane[0]
    normal = plane[1]
    d = point.dot(normal)
    standard_form = np.append(normal,d).tolist()
    return standard_form

def pt_is_inside_poly(pt,Vs):
    for V in Vs:
        if V[1].dot(pt-V[0])>=0:
            return False
    return True

def set_obs_Vs(obs,Vs):
    new_ob_set = []
    for pt in obs:
        if pt_is_inside_poly(pt,Vs):
            new_ob_set.append(pt)
    new_ob_set = np.array(new_ob_set).astype(np.float)
    return new_ob_set  

def check_for_similar_plane(pln,polys,angl_lim):
    v0 = pln[1]
    p0 = pln[0]
    idxs_similar = []
    for i_,plane in enumerate(polys):
        v1 = plane[1]
        p1 = plane[0]
        #if abs(norm(p0-p1))<=near_factor:
        if np.math.atan2(norm(np.cross(v0,v1)),np.dot(v0,v1))<=angl_lim:
            idxs_similar.append(i_)
    return idxs_similar!=[] , idxs_similar
    
def form_hybrid_plane(hypplane,globalpoly,plane_idx):
    new_global_poly = list(globalpoly)
    similar_normals = np.array([new_global_poly.pop(idx_)[0] for idx_ in plane_idx]+hypplane[0])
    new_normal = np.sum(similar_normals, axis=0)/len(similar_normals)
    new_normal = new_normal/norm(new_normal)
    hyb_plane = [hypplane[0],new_normal]
    new_global_poly.append(hyb_plane)
    return new_global_poly



def get_hyperplanes(obs_remain,Cinv,d,angl_lim):
    globalpoly = []
    while(obs_remain!=[]):
        hypplane = E.nearest_hyperplane(Cinv,d,obs_remain)
        if not np.all(hypplane[1]==0): #add it only if the plane has valid normals
            if angl_lim == 0:
                globalpoly.append(hypplane)
            else:
                planes_similar, plane_idx = check_for_similar_plane(hypplane,[globalpoly[len(globalpoly)-1]],angl_lim,0)
                if not planes_similar:
                    globalpoly.append(hypplane)
                else:
                    hypplane = form_hybrid_plane(hypplane,[globalpoly[len(globalpoly)-1]],plane_idx)
                    globalpoly = globalpoly[0:len(globalpoly)-1]
                    globalpoly.append(hypplane)
        obs_retain = []
        for pt in obs_remain:
            if pt_is_inside_poly(pt,globalpoly):
                obs_retain.append(pt)
        obs_remain = obs_retain
    return globalpoly


def add_local_bbox(p1,p2,bbox):
    #P1 and P2 are 3*1 vectors
    #bbox is a l,b of the bbox around the line segment - 2D vector essentially
    if norm(np.array(bbox)) == 0:
        return []
    #get unitvector parallel and normal to the line segment:
    unitvec_parallel = (p2-p1)/norm(p2-p1)
    unitvec_norm = np.array([unitvec_parallel[1],-unitvec_parallel[0],0])

    if norm(unitvec_norm) == 0:
        unitvec_norm = np.array([-1,0,0]).astype(np.float)
    
    unitvec_norm = unitvec_norm/norm(unitvec_norm)  #normalize the vector
    #Get the Unitvectors vertical to the other two , cross product is faster if individually multiplied on python
    unitvec_vert = np.array([0,0,0]).astype(np.float)
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

