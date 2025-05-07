import torch
import yaml
from rl_module.policy_network import PolicyNetwork

class InferenceRunner:
    def __init__(self, model_path, obs_dim=30, action_dim=12):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = PolicyNetwork(obs_dim, action_dim).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

    def predict(self, obs):
        with torch.no_grad():
            obs_tensor = torch.tensor(obs, dtype=torch.float32).to(self.device)
            action = self.model(obs_tensor)
        return action.cpu().numpy()

# Example usage:
# runner = InferenceRunner("trained_model.pth")
# action = runner.predict([0.0]*30)
