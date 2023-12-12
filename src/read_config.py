import yaml
from typing import Any, Mapping
from shapely import Polygon, LinearRing

def _load_config(file_path: str) -> Mapping[str, Any]:
    with open(file_path, "r") as file:
        config: dict[str, Any] = yaml.safe_load(file)
    return config

def read_config_from_yaml(file_path: str):
    config = _load_config(file_path=file_path)

    #  obstacles
    obstacle_list = list(map(list, config["static_obstacles"]))
    #print(obstacle_list)
    obstacles = list(map(Polygon, config["static_obstacles"]))
    boundary = LinearRing(config["boundary"])
    bounds = boundary.bounds
    #print(bounds)
    win_size = [bounds[2] - bounds[0], bounds[3] - bounds[1]]
    #print(win_size)

    # agents
    agents_dict = config["agents"]
    
    return obstacle_list, obstacles, boundary, bounds, win_size, agents_dict