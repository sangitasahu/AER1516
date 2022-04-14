import numpy as np

def get_obs(pcloud,channel):
    if channel ==[]:
        obs_ = np.array(pcloud)
        return obs_
    obs_probability = 0.5
    obs_ = np.array(pcloud)[np.argwhere(np.array(channel)>obs_probability).T[0],:]
    return obs_
