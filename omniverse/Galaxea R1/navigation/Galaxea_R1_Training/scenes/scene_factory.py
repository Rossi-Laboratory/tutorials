
import importlib

def load_scene(scene_name, style='warehouse'):
    module = importlib.import_module(f'Galaxea_R1_Navigation.scenes.{scene_name}')
    class_name = "".join([part.capitalize() for part in scene_name.split("_")]) + "Builder"
    SceneClass = getattr(module, class_name)
    return SceneClass(style=style)
