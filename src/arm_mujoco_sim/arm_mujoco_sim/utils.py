import numpy as np

def clip_joint_value(value, lower_limit, upper_limit):
    """关节值限位（单个关节）"""
    return np.clip(value, lower_limit, upper_limit)

def smooth_interpolate(current, target, max_step, dead_zone=0.001):
    """平滑插值（避免关节突变）"""
    error = target - current
    if abs(error) < dead_zone:
        return target
    step = np.sign(error) * min(max_step, abs(error))
    return current + step

def get_joint_limits(model, joint_num):
    """从Mujoco模型提取关节限位"""
    limits = []
    for i in range(joint_num):
        lower, upper = model.jnt_range[i]
        if np.isinf(lower) or np.isinf(upper):
            limits.append((-np.pi, np.pi))  # 默认限位
        else:
            limits.append((lower, upper))
    return limits