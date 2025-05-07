import torch

def reward_action_smoothness(action, prev_action, weight=0.05):
    return -weight * torch.sum((action - prev_action)**2, dim=-1)
