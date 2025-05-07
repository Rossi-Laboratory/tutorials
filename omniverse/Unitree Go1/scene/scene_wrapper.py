from scene_loader import load_scene_config

def generate_terrain_cfg(scene_name):
    config = load_scene_config(scene_name)
    terrain_type = config.get("terrain_type", "box")
    terrain_cfg = {"type": terrain_type}

    if terrain_type == "box":
        terrain_cfg["friction"] = config.get("friction", 1.0)
    elif terrain_type == "inclined_plane":
        terrain_cfg.update({
            "friction": config.get("friction", 1.0),
            "slope_angle": config.get("slope", 10.0)
        })
    elif terrain_type == "discrete_steps":
        terrain_cfg.update({
            "step_height": config.get("step_height", 0.1),
            "step_spacing": config.get("step_spacing", 0.3)
        })
    elif terrain_type == "noise_heightfield":
        terrain_cfg.update({
            "amplitude": config.get("amplitude", 0.05),
            "frequency": config.get("frequency", 1.5)
        })
    elif terrain_type == "platform_gap":
        terrain_cfg.update({
            "gap_width": config.get("gap_width", 0.2),
            "platform_length": config.get("platform_length", 0.5)
        })
    elif terrain_type == "deformable_surface":
        terrain_cfg.update({
            "softness": config.get("softness", 0.5),
            "damping": config.get("damping", 0.3)
        })
    else:
        raise ValueError(f"Unsupported terrain type: {terrain_type}")

    return terrain_cfg
