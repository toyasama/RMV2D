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

from dataclasses import dataclass, field
from threading import RLock
from typing import Any


@dataclass
class FramesParameters:
    """Class to manage the parameters of the frames.
    Args:
        main_frame (str): The main frame of the visualization.
        sub_frames (list[str]): The list of sub frames to be visualized.
        show_axes (bool): Whether to show the axes of the frames.
        show_frame_names (bool): Whether to show the names of the frames.
        show_connections (bool): Whether to show the connections between the frames.
        axes_length (float): The length of the axes of the frames.
        show_sub_frames (bool): Whether to show the sub frames."""

    _main_frame: str = field(default="", init=False)
    _sub_frames: list[str] = field(default_factory=list, init=False)
    _show_sub_frames: bool = field(default=True, init=False)
    _show_axes: bool = field(default=True, init=False)
    _show_frame_names: bool = field(default=True, init=False)
    _show_connections: bool = field(default=True, init=False)
    _axes_length: float = field(default=0.1, init=False)

    def __dict__(self) -> dict[str, Any]:
        return {
            "frames": {
                "main_frame": self._main_frame,
                "sub_frames": self._sub_frames,
                "show_axes": self._show_axes,
                "show_frame_names": self._show_frame_names,
                "show_connections": self._show_connections,
                "axes_length": self._axes_length,
                "show_sub_frames": self._show_sub_frames,
            }
        }

    def __post_init__(self):
        self._locks = {
            "main_frame": RLock(),
            "sub_frames": RLock(),
            "show_axes": RLock(),
            "show_frame_names": RLock(),
            "show_connections": RLock(),
            "axes_length": RLock(),
            "show_sub_frames": RLock(),
        }

    @property
    def show_sub_frames(self) -> bool:
        with self._locks["show_sub_frames"]:
            return self._show_sub_frames

    @show_sub_frames.setter
    def show_sub_frames(self, value: bool) -> None:
        with self._locks["show_sub_frames"]:
            if not isinstance(value, bool):
                return
            self._show_sub_frames = value

    @property
    def main_frame(self) -> str:
        with self._locks["main_frame"]:
            return self._main_frame

    @main_frame.setter
    def main_frame(self, value: str) -> None:
        with self._locks["main_frame"]:
            self._main_frame = value

    @property
    def sub_frames(self) -> dict[str, dict[str, Any]]:
        with self._locks["sub_frames"]:
            return self._sub_frames.copy()

    @property
    def show_axes(self) -> bool:
        with self._locks["show_axes"]:
            return self._show_axes

    @show_axes.setter
    def show_axes(self, value: bool) -> None:
        if not isinstance(value, bool):
            return
        with self._locks["show_axes"]:
            self._show_axes = value

    def toggleAxes(self) -> None:
        with self._locks["show_axes"]:
            self._show_axes = not self._show_axes

    @property
    def show_frame_names(self) -> bool:
        with self._locks["show_frame_names"]:
            return self._show_frame_names

    @show_frame_names.setter
    def show_frame_names(self, value: bool) -> None:
        if not isinstance(value, bool):
            return
        with self._locks["show_frame_names"]:
            self._show_frame_names = value

    def toggleFrameNames(self) -> None:
        with self._locks["show_frame_names"]:
            self._show_frame_names = not self._show_frame_names

    @property
    def show_connections(self) -> bool:
        with self._locks["show_connections"]:
            return self._show_connections

    @show_connections.setter
    def show_connections(self, value: bool) -> None:
        if not isinstance(value, bool):
            return
        with self._locks["show_connections"]:
            self._show_connections = value

    def toggleConnections(self) -> None:
        with self._locks["show_connections"]:
            self._show_connections = not self._show_connections

    def toggleSubFrames(self) -> None:
        with self._locks["show_sub_frames"]:
            self._show_sub_frames = not self._show_sub_frames

    @property
    def axes_length(self) -> float:
        with self._locks["axes_length"]:
            return self._axes_length

    @axes_length.setter
    def axes_length(self, value: float) -> None:
        if value > 0:
            with self._locks["axes_length"]:
                self._axes_length = value

    def toDict(self) -> dict[str, Any]:
        return self.__dict__()

    def updateFromDict(self, data: dict[str, Any]) -> None:
        """
        Update all parameters from a dictionary if keys match properties.
        """
        for key, value in data.items():
            attr_name = f"_{key}"
            print(f"Updating {attr_name} to {value}")
            if hasattr(self, attr_name):
                lock = self._locks.get(key)
                if lock:
                    with lock:
                        print(f"Updating {attr_name} to {value}")
                        setattr(self, attr_name, value)
            elif key == "sub_frames" and isinstance(value, list):
                print("Updating sub frames to ", value)
                self.updateSubFrame(value)

    def addSubFrame(self, frame_name: str) -> None:
        with self._locks["sub_frames"]:
            if frame_name not in self._sub_frames:
                print("Adding sub frame ", frame_name)
                self._sub_frames.append(frame_name)

    def removeSubFrame(self, frame_name: str) -> None:
        with self._locks["sub_frames"]:
            if frame_name in self._sub_frames:
                self._sub_frames.remove(frame_name)

    def updateSubFrame(self, frame_list: list) -> None:
        with self._locks["sub_frames"]:
            self._sub_frames = frame_list

    def updateByKeyPath(self, parts: list[str], value: Any) -> None:
        if not parts:
            raise ValueError("Empty key path")

        key = parts[0]
        if key == "sub_frames":
            if isinstance(value, list):
                self.updateSubFrame(value)
            elif isinstance(value, str):
                self.addSubFrame(value)
        elif hasattr(self, key):
            setattr(self, key, value)
        else:
            raise KeyError(f"Invalid frames key path: {'.'.join(parts)}")
