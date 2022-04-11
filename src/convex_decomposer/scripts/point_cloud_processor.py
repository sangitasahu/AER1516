import numpy as np

def get_obs(pcloud,channel):
    if channel ==[]:
        return np.array(pcloud)
    obs_probability = 0.5
    obs = []
    
    for i, state in enumerate(channel):
        if state >= obs_probability:
            obs.append(pcloud[i])
    return obs
