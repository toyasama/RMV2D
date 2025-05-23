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
class VisualizationParameters:
    """Class to manage the parameters of the visualization.
    Args:
        width (int): The width of the visualization window.
        height (int): The height of the visualization window.
        fps (int): The frames per second of the visualization.
        draw_grid (bool): Whether to draw the grid in the visualization.
        background_color (dict[str, int]): The background color of the visualization.
        grid_color (dict[str, int]): The color of the grid in the visualization.
        camera (dict[str, dict[str, Any]]): The camera parameters of the visualization.
        grid_spacing (float): The spacing of the grid in the visualization.
    """

    _width: int = field(default=100, init=False)
    _height: int = field(default=100, init=False)
    _fps: int = field(default=10, init=False)
    _draw_grid: bool = field(default=True, init=False)
    _background_color: dict[str, int] = field(
        default_factory=lambda: {"r": 0, "g": 0, "b": 0}, init=False
    )
    _grid_color: dict[str, int] = field(
        default_factory=lambda: {"r": 255, "g": 255, "b": 255}, init=False
    )
    _camera: dict[str, dict[str, Any]] = field(
        default_factory=lambda: {
            "position": {"x": 0, "y": 0, "z": 0, "theta": 0},
            "fov_deg": 60,
        },
        init=False,
    )
    _grid_spacing: float = field(default=1, init=False)

    def __dict__(self) -> dict[str, Any]:
        return {
            "visualizations": {
                "width": self._width,
                "height": self._height,
                "fps": self._fps,
                "draw_grid": self._draw_grid,
                "background_color": self._background_color,
                "grid_color": self._grid_color,
                "camera": self._camera,
                "grid_spacing": self._grid_spacing,
            }
        }

    def __post_init__(self):
        self._locks = {
            "width": RLock(),
            "height": RLock(),
            "fps": RLock(),
            "draw_grid": RLock(),
            "background_color": RLock(),
            "grid_color": RLock(),
            "camera": RLock(),
            "grid_spacing": RLock(),
            "fov": RLock(),
        }

    @property
    def resolution(self) -> tuple[int, int, int]:
        with self._locks["width"], self._locks["height"], self._locks["camera"]:
            return self._width, self._height, self._camera["fov_deg"]

    def setResolution(
        self,
        width: int | None = None,
        height: int | None = None,
        fov: int | None = None,
    ) -> None:
        with self._locks["width"], self._locks["height"], self._locks["camera"]:
            if width is not None:
                self._width = width
            if height is not None:
                self._height = height
            if fov is not None:
                self._camera["fov_deg"] = fov

    @property
    def width(self) -> int:
        with self._locks["width"]:
            return self._width

    @width.setter
    def width(self, value: int) -> None:
        if value > 0:
            with self._locks["width"]:
                self._width = value

    @property
    def height(self) -> int:
        with self._locks["height"]:
            return self._height

    @height.setter
    def height(self, value: int) -> None:
        print(f"height {value}")
        if value > 0:
            with self._locks["height"]:
                print(f"height {value}")
                self._height = value

    @property
    def fps(self) -> int:
        with self._locks["fps"]:
            return self._fps

    @fps.setter
    def fps(self, value: int) -> None:
        if value > 0:
            with self._locks["fps"]:
                self._fps = value

    @property
    def background_color(self) -> dict[str, int]:
        with self._locks["background_color"]:
            return self._background_color.copy()

    def updateBackgroundColor(
        self, r: int | None = None, g: int | None = None, b: int | None = None
    ) -> None:
        r = int(r) if isinstance(r, (str, float)) else r
        g = int(g) if isinstance(g, (str, float)) else g
        b = int(b) if isinstance(b, (str, float)) else b
        with self._locks["background_color"]:
            if isinstance(r, int):
                self._background_color["r"] = r
            if isinstance(g, int):
                self._background_color["g"] = g
            if isinstance(b, int):
                self._background_color["b"] = b

    @property
    def camera_position(self) -> tuple[float, float, float, float]:
        with self._locks["camera"]:
            return tuple(
                self._camera["position"][key] for key in ["x", "y", "z", "theta"]
            )

    def updateCameraPosition(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        theta: float | None = None,
    ) -> None:
        x = float(x) if isinstance(x, str) else x
        y = float(y) if isinstance(y, str) else y
        z = float(z) if isinstance(z, str) else z
        theta = float(theta) if isinstance(theta, str) else theta
        with self._locks["camera"]:
            if x is not None:
                self._camera["position"]["x"] = x
            if y is not None:
                self._camera["position"]["y"] = y
            if z is not None:
                self._camera["position"]["z"] = z
            if theta is not None:
                self._camera["position"]["theta"] = theta

    def updateCameraPositionX(self, x: float) -> None:
        if not isinstance(x, (int, float)):
            try:
                x = float(x)
            except ValueError:
                return
        with self._locks["camera"]:
            self._camera["position"]["x"] = x

    def updateCameraPositionY(self, y: float) -> None:
        if not isinstance(y, (int, float)) and y <= 0:
            try:
                y = float(y)
            except ValueError:
                return
        with self._locks["camera"]:
            self._camera["position"]["y"] = y

    def updateCameraPositionZ(self, z: float) -> None:
        if not isinstance(z, (int, float)):
            try:
                z = float(z)
            except ValueError:
                return
        with self._locks["camera"]:
            self._camera["position"]["z"] = z

    def updateCameraPositionTheta(self, theta: float) -> None:
        if not isinstance(theta, (int, float)):
            try:
                theta = float(theta)
            except ValueError:
                return
        with self._locks["camera"]:
            self._camera["position"]["theta"] = theta

    def updateCameraFOV(self, fov: int) -> None:
        if not isinstance(fov, int) or not 10 <= fov <= 180:
            try:
                fov = int(fov)
            except ValueError:
                return
        with self._locks["camera"]:
            self._camera["fov_deg"] = fov

    @property
    def fov(self) -> int:
        with self._locks["camera"]:
            return self._camera["fov_deg"]

    @fov.setter
    def fov(self, value: int) -> None:
        if 10 <= value <= 180:
            with self._locks["camera"]:
                self._camera["fov_deg"] = value

    @property
    def grid_spacing(self) -> float:
        with self._locks["grid_spacing"]:
            return self._grid_spacing

    @grid_spacing.setter
    def grid_spacing(self, value: float) -> None:
        if value > 0:
            with self._locks["grid_spacing"]:
                self._grid_spacing = value

    @property
    def draw_grid(self) -> bool:
        with self._locks["draw_grid"]:
            return self._draw_grid

    @draw_grid.setter
    def draw_grid(self, value: bool) -> None:
        with self._locks["draw_grid"]:
            self._draw_grid = value

    @property
    def grid_color(self) -> dict[str, int]:
        with self._locks["grid_color"]:
            return self._grid_color.copy()

    def updateGridColor(
        self, r: int | None = None, g: int | None = None, b: int | None = None
    ) -> None:
        r = int(r) if isinstance(r, (str, float)) else r
        g = int(g) if isinstance(g, (str, float)) else g
        b = int(b) if isinstance(b, (str, float)) else b
        with self._locks["grid_color"]:
            if isinstance(r, int):
                self._grid_color["r"] = r
            if isinstance(g, int):
                self._grid_color["g"] = g
            if isinstance(b, int):
                self._grid_color["b"] = b

    def updateGridSpacing(self, spacing: float) -> None:
        if not isinstance(spacing, (int, float)) or spacing <= 0:
            try:
                spacing = float(spacing)
            except ValueError:
                return
        with self._locks["grid_spacing"]:
            self._grid_spacing = spacing

    def toggleDrawGrid(self):
        with self._locks["draw_grid"]:
            self._draw_grid = not self._draw_grid

    def toDict(self) -> dict[str, Any]:
        return self.__dict__()

    def updateFromDict(self, data: dict[str, Any]) -> None:
        """
        Update all visualization parameters from a dictionary if keys match properties.
        """
        for key, value in data.items():
            attr_name = f"_{key}"

            if hasattr(self, attr_name):
                lock = self._locks.get(key)
                if lock:
                    with lock:
                        setattr(self, attr_name, value)

            elif key == "background_color" and isinstance(value, dict):
                self.updateBackgroundColor(**value)

            elif key == "grid_color" and isinstance(value, dict):
                self.updateGridColor(**value)

            elif key == "camera" and isinstance(value, dict):
                if "position" in value:
                    self.updateCameraPosition(**value["position"])
                if "fov_deg" in value:
                    self.updateCameraFOV(value["fov_deg"])

    def updateByKeyPath(self, parts: list[str], value: Any) -> None:
        if not parts:
            raise ValueError("Empty key path")

        key = parts[0]
        if key in ("background_color", "grid_color", "camera") and len(parts) >= 2:
            sub_key = parts[1]
            if key == "background_color":
                self.updateBackgroundColor(**{sub_key: value})
            elif key == "grid_color":
                self.updateGridColor(**{sub_key: value})
            elif key == "camera":
                if sub_key == "fov_deg":
                    self.updateCameraFOV(value)
                elif sub_key == "position" and len(parts) == 3:
                    pos_key = parts[2]
                    self.updateCameraPosition(**{pos_key: value})
                else:
                    raise KeyError(f"Invalid camera key path: {'.'.join(parts)}")
        elif hasattr(self, key):
            setattr(self, key, value)
        else:
            raise KeyError(f"Invalid visualization key path: {'.'.join(parts)}")
