import numpy as np

def clip_joint_value(value, lower_limit, upper_limit):

    return np.clip(value, lower_limit, upper_limit)

def smooth_interpolate(current, target, max_step, dead_zone=0.001):
  
    error = target - current
    if abs(error) < dead_zone:
        return target
    step = np.sign(error) * min(max_step, abs(error))
    return current + step

def get_joint_limits(model, joint_num):

    limits = []
    for i in range(joint_num):
        lower, upper = model.jnt_range[i]
        if np.isinf(lower) or np.isinf(upper):
            limits.append((-np.pi, np.pi)) 
        else:
            limits.append((lower, upper))
    return limits