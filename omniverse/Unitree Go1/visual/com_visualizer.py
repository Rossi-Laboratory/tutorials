import matplotlib.pyplot as plt
import numpy as np

def visualize_com_trajectory(com_positions):
    com_positions = np.array(com_positions)
    fig = plt.figure(figsize=(6, 6))
    plt.plot(com_positions[:, 0], com_positions[:, 1], marker='o', label='COM Trajectory')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Center of Mass (COM) Trajectory')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

# Example usage:
# visualize_com_trajectory([[0, 0], [0.1, 0.05], [0.2, 0.1]])
