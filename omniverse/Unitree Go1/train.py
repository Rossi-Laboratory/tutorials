import argparse
import yaml
import torch
from torch.utils.tensorboard import SummaryWriter
from curriculum.com_shift_curriculum import update_com_shift
from goal.goal_reward import reward_goal_velocity
from rl_module.energy_reward import reward_energy
from rl_module.action_penalty import reward_action_smoothness
from utils.obs_normalizer import RunningMeanStd
from scene.scene_wrapper import generate_terrain_cfg

class DummyEnv:
    def __init__(self, terrain):
        self.terrain = terrain  # scene config injected here
    def reset(self):
        return torch.zeros(30)
    def step(self, a):
        return torch.zeros(30), 1.0, False, {}

def train(cfg_path):
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # Load terrain
    terrain_name = cfg.get("env", {}).get("scene_name", "flat")
    terrain_cfg = generate_terrain_cfg(terrain_name)
    print(f"[INFO] Loaded terrain: {terrain_cfg}")

    env = DummyEnv(terrain_cfg)
    obs = env.reset()
    writer = SummaryWriter()

    use_obs_norm = cfg.get('features', {}).get('obs_normalization', False)
    use_smooth = cfg.get('features', {}).get('action_smooth_penalty', False)
    use_residual = cfg.get('features', {}).get('residual_control', False)

    obs_rms = RunningMeanStd(shape=obs.shape) if use_obs_norm else None
    prev_action = torch.zeros(12)

    for step in range(1000):
        obs_np = obs.numpy()
        if use_obs_norm:
            obs_rms.update(obs_np[None, :])
            obs_tensor = torch.tensor(obs_rms.normalize(obs_np), dtype=torch.float32)
        else:
            obs_tensor = obs

        policy_action = torch.randn(12)
        action = policy_action + 0.0
        if use_residual:
            action += torch.zeros_like(policy_action)

        next_obs, _, _, _ = env.step(action)
        r1 = reward_goal_velocity(torch.rand(3), torch.rand(3))
        r2 = reward_energy(action)
        r3 = reward_action_smoothness(action, prev_action) if use_smooth else 0.0
        shift = update_com_shift(step)

        writer.add_scalar("Reward/goal", r1.mean().item(), step)
        writer.add_scalar("Reward/energy", r2.mean().item(), step)
        if use_smooth:
            writer.add_scalar("Reward/smooth", r3.mean().item(), step)
        writer.add_scalar("COM/shift", shift, step)

        prev_action = action.detach()
        obs = next_obs

    writer.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, required=True)
    args = parser.parse_args()
    train(args.cfg)
