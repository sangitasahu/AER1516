import numpy as np
from scipy.spatial.transform import Rotation

def get_R_from2vectors(vec1, vec2):
    """get rotation matrix between two vectors using scipy"""
    vec1 = np.reshape(vec1, (1, -1))[0]
    vec2 = np.reshape(vec2, (1, -1))[0]    
    v1_crossv2 = np.cross(vec1,vec2)
    norm_v1crosv2 = np.linalg.norm(v1_crossv2)
    q = [0,0,0,1]
    if norm_v1crosv2:
        theta = np.arctan2(norm_v1crosv2,vec1.dot(vec2))    
        n= v1_crossv2/norm_v1crosv2
        xyz = n.dot(np.sin(theta/2)).tolist()
        q =xyz+[np.cos(theta/2)]
    np.set_printoptions(precision=3)
    r = Rotation.from_quat(q)
    return r.as_dcm()

def sort_pts(pts):
    if pts ==[]:
        return pts
    pts = np.asarray(pts)
    avg = np.average(pts,0)
    pts_valued = []
    for pt in pts:
        theta = np.arctan2(pt[1]-avg[1],pt[0]-avg[0])
        pts_valued.append([theta,pt[0],pt[1]])
    pts_valued = np.array(pts_valued)
    sorted_pts = pts_valued[pts_valued[:, 0].argsort()]
    sorted_pts = sorted_pts[:,1:3]
    return sorted_pts

def get_intersect_pt(v1,v2):
    a1 = -v1[0][1]
    b1 = v1[0][0]
    c1 = a1*v1[1][0]+b1*v1[1][1]

    a2 = -v2[0][1]
    b2 = v2[0][0]
    c2 = a2*v2[1][0]+b2*v2[1][1]

    if abs(a1 * b2 - a2 * b1)<=1e-10 or abs(a2 * b1 - a1 * b2)<=1e-10:
        return False, []
    x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1)
    y = (c1 * a2 - c2 * a1) / (a2 * b1 - a1 * b2)
    return True,[x,y]
    
def intersections_on_line(lines):
    pts = []
    for i in range(0,len(lines)):
        line1 = lines[i]
        for j in range(i+1,len(lines)):
            line2 = lines[j]
            state, pt_intersect = get_intersect_pt(line1,line2)
            if state==True:
                pts.append(pt_intersect)                
    return pts

def signed_dist_pt_plane(pt,V):
    n = V[1]
    p = V[0]
    distance = n.dot(pt-p)
    return distance

def pt_is_inside_poly(pt,Vs):
    for V in Vs:
        if signed_dist_pt_plane(pt,V)>=1e-10:
            return False
    return True

def calculate_vertices(poly_h):
    vertices = []
    vertices_planar=[]
    for i,plane1 in enumerate(poly_h):
        p1 = plane1[0]
        n1 = plane1[1]
        R = get_R_from2vectors([0,0,1],n1)
        lines = []
        for j,plane2 in enumerate(poly_h):
            if (j == i): #eliminate possibility of the plane1 --same plane being used
                continue
            n2 = plane2[1]
            p2 = plane2[0]
            rotated_normal = R.T.dot(n2)
            b = p2.dot(n2) - n2.dot(p1)
            v = np.array([0,0,1])
            v = np.cross(v,rotated_normal)[0:2]
            if (rotated_normal[1] !=0):
                p = np.array([0,b/rotated_normal[1]])    
            elif (rotated_normal[0] != 0 ):
                p = np.array([b/rotated_normal[0],0])
            else:
                continue
            lines.append([v,p])
        pts = intersections_on_line(lines)
        pts_inside = []
        for pt in pts:
            p = R.dot(np.array([pt[0],pt[1],0]))+p1
            if pt_is_inside_poly(p,poly_h):
                pts_inside.append(pt)
        if len(pts_inside)>2:
            pts_inside = sort_pts(pts_inside)
            valid_pts = []
            valid_pts_raw = [] #Unused Debug only
            for ptr in pts_inside:
                valid_pts.append(R.dot(np.array([ptr[0],ptr[1],0]))+p1)
                valid_pts_raw.append(np.array([ptr[0],ptr[1],0]))#Unused > Debug only
            vertices.append(valid_pts)
            vertices_planar.append(valid_pts_raw) #Unused
    return vertices   

def merge_planes(poly,d):
    plane_normals = ([plane[1].tolist() for plane in poly])
    idx_check = []
    
    #for i in range(0,len(plane_normals)-1):
    #    idx_list = [i]
    #    for j in range(i+1,len(plane_normals)):
    #        if i!=j:
    #            v0 = plane_normals[i]
    #            v1 = plane_normals[j]
    #            angle = np.math.atan2(np.linalg.norm(np.cross(v0,v1)),np.dot(v0,v1))
    #            if np.degrees(angle)<=20:
    #                idx_list.append(j)
    #    nrms = [plane_normals[id] for id in idx_list]
    #    new_norm = np.divide((np.sum(np.array(nrms),axis = 0)),len(idx_list))
    #    new_poly.append([poly[i][0],np.array(new_norm)])

    
    return poly

def eliminate_redundant_planes(poly,d):
    plane_normals = ([plane[1].tolist() for plane in poly])
    all_idxs = []
    for i in range(0,len(plane_normals)):
        idx = []
        for j in range(0,len(plane_normals)):
            diff = np.abs(np.array(plane_normals[i]) - np.array(plane_normals[j]))
            if np.all(diff <=1e-10)  and i!=j:
                if idx==[]:
                    idx.append(i)
                idx.append(j)
        idx.sort()
        if idx!=[] and idx not in all_idxs:
            all_idxs.append(idx)
    idxs_to_eliminate = []

    for plane_indeces in all_idxs:
        set_of_planes = [poly[p] for p in plane_indeces]
        for i,plane in enumerate(set_of_planes):
            otherplanes = list(set_of_planes)
            otherplanes.pop(i)
            if not pt_is_inside_poly(plane[0],otherplanes):
                idxs_to_eliminate.append(plane_indeces[i])
    new_poly = []
    for i in range(0,len(poly)):
        if i not in idxs_to_eliminate:
            new_poly.append(poly[i])
    
    new_poly = merge_planes(new_poly,d)

    return new_poly

    

    