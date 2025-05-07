import torch
from torch.utils.data import DataLoader, Dataset
import torch.nn.functional as F

class RealWorldDataset(Dataset):
    def __init__(self, data_file):
        data = torch.load(data_file)
        self.obs = data["obs"]
        self.actions = data["actions"]

    def __len__(self):
        return len(self.obs)

    def __getitem__(self, idx):
        return self.obs[idx], self.actions[idx]

def adapt(policy, data_file, lr=3e-4, epochs=5):
    dataset = RealWorldDataset(data_file)
    loader = DataLoader(dataset, batch_size=64, shuffle=True)
    optimizer = torch.optim.Adam(policy.parameters(), lr=lr)
    policy.train()
    for epoch in range(epochs):
        for obs, target_act in loader:
            pred = policy(obs)
            loss = F.mse_loss(pred, target_act)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        print(f"Epoch {epoch+1}: loss={loss.item():.4f}")
