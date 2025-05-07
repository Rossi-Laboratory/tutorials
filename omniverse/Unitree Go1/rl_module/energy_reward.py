
import torch

def reward_energy(torque, weight=0.01):
    return -weight * torch.sum(torque ** 2, dim=1)
