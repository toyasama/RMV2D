# MIT License
#
# Copyright (c) 2025 [Tognama Emery-donald]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from dataclasses import dataclass
from typing import Any

import yaml

from .frame_parameter import FramesParameters
from .visualization_parameter import VisualizationParameters


@dataclass
class RmvParameters:
    def __init__(self, path: str):
        self.__path = path
        data = self._loadYaml()
        print(data)

        self.visualization = VisualizationParameters()
        self.frames = FramesParameters()

        visualization_data = data.get("RMV", {}).get("visualizations", {})
        frames_data = data.get("RMV", {}).get("frames", {})

        self._update_visualization(visualization_data)
        self._update_frames(frames_data)

    def _loadYaml(self) -> dict:
        """Load the YAML file and apply default values if necessary.

        Returns:
            The loaded data as a dictionary.

        """
        if not os.path.exists(self.__path):
            print(f"File {self.__path} not found, creating an empty file.")
            return {}

        try:
            with open(self.__path, encoding="utf-8") as file:
                data = yaml.safe_load(file) or {}
        except Exception as e:
            print(f"Error reading the file: {e}")
            return {}

        return self._applyDefaults(data)

    def _applyDefaults(self, data: dict) -> dict:
        """Apply default values if some keys are missing."""
        defaults = {
            "RMV": {
                "visualizations": {},
                "frames": {},
            },
        }
        return self._mergeDefaults(defaults, data)

    def _mergeDefaults(self, defaults: dict, data: dict) -> dict:
        """Merge default values with those loaded from the file."""
        if isinstance(defaults, dict) and isinstance(data, dict):
            for key, value in defaults.items():
                if key not in data:
                    print(f"Missing key: {key}, assigning default value.")
                    data[key] = value
                else:
                    data[key] = self._mergeDefaults(value, data[key])
        return self.convertData(data)

    def convertData(self, data: Any) -> Any:
        if isinstance(data, str):
            if data.lower() in ["true", "false"]:
                return data.lower() == "true"
        return data

    def _update_visualization(self, data: dict[str, Any]) -> None:
        if "width" in data:
            self.visualization.width = data["width"]
        if "height" in data:
            self.visualization.height = data["height"]
        if "fps" in data:
            self.visualization.fps = data["fps"]
        if "grid_spacing" in data:
            self.visualization.grid_spacing = data["grid_spacing"]
        if "draw_grid" in data:
            self.visualization.draw_grid = data["draw_grid"]
        if "grid_color" in data:
            self.visualization.updateGridColor(**data["grid_color"])
        if "background_color" in data:
            self.visualization.updateBackgroundColor(**data["background_color"])
        if "camera" in data and "position" in data["camera"]:
            self.visualization.updateCameraPosition(**data["camera"]["position"])
        if "camera" in data and "fov_deg" in data["camera"]:
            self.visualization.fov = data["camera"]["fov_deg"]

    def _update_frames(self, data: dict[str, Any]) -> None:
        if "main_frame" in data:
            self.frames.main_frame = data["main_frame"]
        if "sub_frames" in data:
            if isinstance(data["sub_frames"], list):
                self.frames.updateSubFrame(data["sub_frames"])
        if "show_axes" in data:
            (
                self.frames.toggleAxes()
                if self.frames.show_axes != data["show_axes"]
                else None
            )
        if "show_frame_names" in data:
            (
                self.frames.toggleFrameNames()
                if self.frames.show_frame_names != data["show_frame_names"]
                else None
            )
        if "show_connections" in data:
            (
                self.frames.toggleConnections()
                if self.frames.show_connections != data["show_connections"]
                else None
            )
        if "axes_length" in data:
            self.frames.axes_length = data["axes_length"]
        if "show_sub_frames" in data:
            (
                self.frames.toggleSubFrames()
                if self.frames.show_sub_frames != data["show_sub_frames"]
                else None
            )

    def get(self, *keys):
        """Retrieve a value from the given keys."""
        value = self.data
        for key in keys:
            if key in value:
                value = value[key]
            else:
                print(f"Key {'.'.join(keys)} not found, returning default value.")
                return None
        return value

    def set(self, value, *keys):
        """Update a value and save it to the YAML file."""
        d = self.data
        for key in keys[:-1]:
            if key not in d:
                d[key] = {}
            d = d[key]
        d[keys[-1]] = value
        self._saveYaml()

    def _saveYaml(self):
        """Save the updated data to the YAML file."""
        try:
            with open(self.__path, "w", encoding="utf-8") as file:
                data = self.frames.toDict()
                data.update(self.visualization.toDict())
                yaml.dump(
                    {"RMV": data}, file, default_flow_style=False, allow_unicode=True
                )
        except Exception as e:
            print(f"Error writing to the file: {e}")
            return
        print("Data saved successfully.")

    def save(self):
        """Save the current data to the YAML file."""
        self._saveYaml()
