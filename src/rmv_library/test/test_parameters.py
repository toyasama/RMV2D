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

import pytest
from rmv_library.parameters.params import FramesParameters
from rmv_library.parameters.visualization_parameter import VisualizationParameters
import pytest
import yaml
import os
from tempfile import NamedTemporaryFile
from rmv_library.parameters.params import RmvParameters


@pytest.fixture
def params():
    return FramesParameters()


@pytest.fixture
def visParams():
    return VisualizationParameters()


@pytest.fixture
def sample_yaml_data():
    return {
        "RMV": {
            "visualizations": {
                "width": 800,
                "height": 600,
                "fps": 30,
                "draw_grid": False,
                "grid_spacing": 2.0,
                "background_color": {"r": 10, "g": 20, "b": 30},
                "grid_color": {"r": 200, "g": 150, "b": 100},
                "camera": {
                    "position": {"x": 1.0, "y": 2.0, "z": 3.0, "theta": 0.5},
                    "fov_deg": 90,
                },
            },
            "frames": {
                "main_frame": "world",
                "sub_frames": ["base_link", "camera_link"],
                "show_axes": False,
                "show_frame_names": False,
                "show_connections": True,
                "axes_length": 2.5,
                "show_sub_frames": False,
            },
        }
    }


@pytest.fixture
def yaml_file_path(sample_yaml_data):
    with NamedTemporaryFile(mode="w+", delete=False) as tmp:
        yaml.dump(sample_yaml_data, tmp)
        tmp_path = tmp.name
    yield tmp_path
    os.remove(tmp_path)


@pytest.mark.timeout(5)
def testDefaultValues(params):
    assert params.main_frame == ""
    assert params.sub_frames == []
    assert params.show_sub_frames is True
    assert params.show_axes is True
    assert params.show_frame_names is True
    assert params.show_connections is True
    assert params.axes_length == 0.1


@pytest.mark.timeout(5)
def testMainFrameSetter(params):
    params.main_frame = "world"
    assert params.main_frame == "world"


@pytest.mark.timeout(5)
def testAddSubFrame(params):
    params.addSubFrame("frame1")
    params.addSubFrame("frame2")
    assert "frame1" in params.sub_frames
    assert "frame2" in params.sub_frames


@pytest.mark.timeout(5)
def testRemoveSubFrame(params):
    params.addSubFrame("frame1")
    params.removeSubFrame("frame1")
    assert "frame1" not in params.sub_frames


@pytest.mark.timeout(5)
def testUpdateSubFrame(params):
    params.updateSubFrame(["a", "b", "c"])
    assert params.sub_frames == ["a", "b", "c"]


@pytest.mark.timeout(5)
def testToggleShowAxes(params):
    original = params.show_axes
    params.toggleAxes()
    assert params.show_axes != original


@pytest.mark.timeout(5)
def testToggleShowFrameNames(params):
    original = params.show_frame_names
    params.toggleFrameNames()
    assert params.show_frame_names != original


@pytest.mark.timeout(5)
def testToggleShowConnections(params):
    original = params.show_connections
    params.toggleConnections()
    assert params.show_connections != original


@pytest.mark.timeout(5)
def testToggleShowSubFrames(params):
    original = params.show_sub_frames
    params.toggleSubFrames()
    assert params.show_sub_frames != original


@pytest.mark.timeout(5)
def testSetAxesLength(params):
    params.axes_length = 0.5
    assert params.axes_length == 0.5


@pytest.mark.timeout(5)
def testRejectInvalidAxesLength(params):
    old_value = params.axes_length
    params.axes_length = -1.0
    assert params.axes_length == old_value


@pytest.mark.timeout(5)
def testToDictStructure(params):
    d = params.toDict()
    assert "frames" in d
    assert isinstance(d["frames"], dict)
    assert d["frames"]["main_frame"] == ""
    assert d["frames"]["show_axes"] is True


@pytest.mark.timeout(5)
def testDefaultValues(visParams):
    assert visParams.width == 100
    assert visParams.height == 100
    assert visParams.fps == 10
    assert visParams.grid_spacing == 1
    assert visParams.fov == 60
    assert visParams.background_color == {"r": 0, "g": 0, "b": 0}
    assert visParams.grid_color == {"r": 255, "g": 255, "b": 255}
    assert visParams.draw_grid is True


@pytest.mark.timeout(5)
def testSetResolution(visParams):
    visParams.setResolution(width=800, height=600, fov=90)
    assert visParams.resolution == (800, 600, 90)


@pytest.mark.timeout(5)
def testSettersWithValidation(visParams):
    visParams.width = -1  # ignored
    visParams.height = -50  # ignored
    visParams.fps = 0  # ignored

    assert visParams.width > 0
    assert visParams.height > 0
    assert visParams.fps > 0


@pytest.mark.timeout(5)
def testSetIndividualProperties(visParams):
    visParams.width = 200
    visParams.height = 150
    visParams.fps = 24
    visParams.fov = 75
    visParams.grid_spacing = 2.5

    assert visParams.width == 200
    assert visParams.height == 150
    assert visParams.fps == 24
    assert visParams.fov == 75
    assert visParams.grid_spacing == 2.5


@pytest.mark.timeout(5)
def testToggleDrawGrid(visParams):
    original = visParams.draw_grid
    visParams.toggleDrawGrid()
    assert visParams.draw_grid != original


@pytest.mark.timeout(5)
def testUpdateBackgroundColor(visParams):
    visParams.updateBackgroundColor(r=100, g=150, b=200)
    assert visParams.background_color == {"r": 100, "g": 150, "b": 200}


@pytest.mark.timeout(5)
def testUpdateGridColor(visParams):
    visParams.updateGridColor(r=10, g=20, b=30)
    assert visParams.grid_color == {"r": 10, "g": 20, "b": 30}


@pytest.mark.timeout(5)
def testUpdateCameraPosition(visParams):
    visParams.updateCameraPosition(x=1.5, y=2.5, z=3.5, theta=0.75)
    assert visParams.camera_position == (1.5, 2.5, 3.5, 0.75)


@pytest.mark.timeout(5)
def testUpdateCameraPositionIndividual(visParams):
    visParams.updateCameraPositionX(10.0)
    visParams.updateCameraPositionY(20.0)
    visParams.updateCameraPositionZ(30.0)
    visParams.updateCameraPositionTheta(1.57)
    assert visParams.camera_position == (10.0, 20.0, 30.0, 1.57)


@pytest.mark.timeout(5)
def testUpdateCameraFOV(visParams):
    visParams.updateCameraFOV(120)
    assert visParams.fov == 120


@pytest.mark.timeout(5)
def testRejectInvalidFOV(visParams):
    old_fov = visParams.fov
    visParams.updateCameraFOV("invalid")
    assert visParams.fov == old_fov

    visParams.fov = 999  # out of range
    assert visParams.fov == old_fov


@pytest.mark.timeout(5)
def testUpdateGridSpacing(visParams):
    visParams.updateGridSpacing(0.5)
    assert visParams.grid_spacing == 0.5


@pytest.mark.timeout(5)
def testToDictStructure(visParams):
    d = visParams.toDict()
    assert "visualizations" in d
    v = d["visualizations"]
    assert isinstance(v, dict)
    assert v["width"] == visParams.width
    assert v["height"] == visParams.height
    assert v["fps"] == visParams.fps
    assert v["draw_grid"] == visParams.draw_grid


@pytest.mark.timeout(5)
def testRmvParametersLoading(yaml_file_path):
    rmv = RmvParameters(yaml_file_path)

    v = rmv.visualization
    f = rmv.frames

    # Visualization checks
    assert v.width == 800
    assert v.height == 600
    assert v.fps == 30
    assert v.draw_grid is False
    assert v.grid_spacing == 2.0
    assert v.background_color == {"r": 10, "g": 20, "b": 30}
    assert v.grid_color == {"r": 200, "g": 150, "b": 100}
    assert v.camera_position == (1.0, 2.0, 3.0, 0.5)
    assert v.fov == 90

    # Frame checks
    assert f.main_frame == "world"
    assert f.sub_frames == ["base_link", "camera_link"]
    assert f.show_axes is False
    assert f.show_frame_names is False
    assert f.show_connections is True
    assert f.axes_length == 2.5
    assert f.show_sub_frames is False


@pytest.mark.timeout(5)
def testRmvParametersSaveAndReload(yaml_file_path):
    rmv = RmvParameters(yaml_file_path)

    rmv.visualization.setResolution(width=1280, height=720)
    rmv.visualization.updateCameraFOV(110)
    rmv.frames.main_frame = "map"
    rmv.frames.toggleSubFrames()

    rmv.save()

    reloaded = RmvParameters(yaml_file_path)
    assert reloaded.visualization.width == 1280
    assert reloaded.visualization.height == 720
    assert reloaded.visualization.fov == 110
    assert reloaded.frames.main_frame == "map"
    assert reloaded.frames.show_sub_frames is True


@pytest.mark.timeout(5)
def testMissingFile():
    rmv = RmvParameters("/path/to/nonexistent/file.yaml")
    assert rmv.visualization.width == 100
