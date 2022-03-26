import numpy as np

def get_obs(pcloud,channel):
    if channel ==[]:
        return np.array(pcloud)
    cloud_vector = np.array(pcloud)
    cloud_channel = np.array(channel)
    searchval = 1
    occupancy_indices = np.where(cloud_channel == searchval)[0]
    obs = cloud_vector[occupancy_indices]
    return obs
