import torch

def reward_goal_velocity(vel_actual, vel_goal, weight=1.0):
    return -weight * torch.sum((vel_actual - vel_goal) ** 2, dim=1)
