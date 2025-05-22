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

from dataclasses import dataclass
from typing import Any


from .frame_parameter import FramesParameters
from .visualization_parameter import VisualizationParameters
from .yaml_handler import YamlHandler
import json


@dataclass
class RmvParameters:
    """
    Class to manage the parameters of the RMV library.
    It loads the parameters from a YAML file and provides methods to access and modify them.
    Args:
        path (str): Path to the YAML file containing the parameters.
    """

    def __init__(self, path: str):
        self.yaml_handler = YamlHandler(path)
        data = self.yaml_handler.loadYaml()

        self.visualization = VisualizationParameters()
        self.frames = FramesParameters()

        visualization_data = data.get("RMV", {}).get("visualizations", {})
        frames_data = data.get("RMV", {}).get("frames", {})

        self._update_visualization(visualization_data)
        self._update_frames(frames_data)

    def __str__(self) -> str:
        dict = self.visualization.toDict()
        dict.update(self.frames.toDict())
        return json.dumps(dict)

    def update(self, data: dict[str, Any]) -> None:
        """Update parameters from a dictionary."""
        self._update_visualization(data)
        self._update_frames(data)

    def updateByKeyPath(self, key_path: str, value: Any) -> None:
        """
        Update a deeply nested parameter using a dotted key path.
        Example: "visualizations.background_color.b" = 25
        """
        print(f"Updating parameter at key path: {key_path} with value: {value}")
        parts = key_path.split(".")
        if not parts or parts[0] != "visualizations" and parts[0] != "frames":
            raise ValueError(f"Invalid key path: {key_path}")

        section = parts[0]
        attr = self.visualization if section == "visualizations" else self.frames

        if hasattr(attr, "updateByKeyPath"):
            print(f"Updating {section} with key path: {parts[1:]}")
            attr.updateByKeyPath(parts[1:], value)
        else:
            raise AttributeError(f"Section '{section}' does not support key updates")

    def _update_visualization(self, data: dict[str, Any]) -> None:
        """Update visualization parameters from dictionary."""
        self.visualization.updateFromDict(data)

    def _update_frames(self, data: dict[str, Any]) -> None:
        """Update frames parameters from dictionary."""
        self.frames.updateFromDict(data)

    def save(self):
        """Save the updated parameters to the YAML file."""
        self.yaml_handler.save(self.frames, self.visualization)
