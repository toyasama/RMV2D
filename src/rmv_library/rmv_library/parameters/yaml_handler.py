import yaml

import os

from typing import Any


from .frame_parameter import FramesParameters
from .visualization_parameter import VisualizationParameters


class YamlHandler:
    """
    Class to handle YAML file operations.
    It provides methods to load and save data from/to a YAML file.
    """

    def __init__(self, path: str):
        self.__path = path

    def loadYaml(self) -> dict:
        if not os.path.exists(self.__path):
            raise FileNotFoundError(
                f"File {self.__path} not found, creating an empty file."
            )

        try:
            with open(self.__path, encoding="utf-8") as file:
                data = yaml.safe_load(file) or {}
        except Exception as e:
            raise (f"Error reading the file: {e}")
        return self._applyDefaults(data)

    def _applyDefaults(self, data: dict) -> dict:
        defaults = {
            "RMV": {
                "visualizations": {},
                "frames": {},
            },
        }
        return self._mergeDefaults(defaults, data)

    def _mergeDefaults(self, defaults: dict, data: dict) -> dict:
        if isinstance(defaults, dict) and isinstance(data, dict):
            for key, value in defaults.items():
                if key not in data:
                    print(f"Missing key: {key}, assigning default value.")
                    data[key] = value
                else:
                    data[key] = self._mergeDefaults(value, data[key])
        return self._convertData(data)

    def _convertData(self, data: Any) -> Any:
        if isinstance(data, str):
            if data.lower() in ["true", "false"]:
                return data.lower() == "true"
        return data

    def save(self, frames: FramesParameters, visualization: VisualizationParameters):
        """Save the updated data to the YAML file."""
        try:
            with open(self.__path, "w", encoding="utf-8") as file:
                data = frames.toDict()
                data.update(visualization.toDict())
                yaml.dump(
                    {"RMV": data}, file, default_flow_style=False, allow_unicode=True
                )
        except Exception as e:
            print(f"Error writing to the file: {e}")
            return
        print("Data saved successfully.")
