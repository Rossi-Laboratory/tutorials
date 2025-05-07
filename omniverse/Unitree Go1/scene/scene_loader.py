import yaml
import os

def load_scene_config(scene_name, scene_dir="scene"):
    filename = os.path.join(scene_dir, f"terrain_{scene_name}.yaml")
    if not os.path.exists(filename):
        raise FileNotFoundError(f"Scene config not found: {filename}")
    with open(filename, 'r') as f:
        config = yaml.safe_load(f)
    return config['scene']
