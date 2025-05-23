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

import numpy as np
import pytest

from rmv_library.utils.drawers_tools.line_strip import LineStripDrawer
from rmv_library.utils.camera import CameraManager
from rmv_library.parameters.params import VisualizationParameters


class DummyPosition:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class DummyOrientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class DummyScale:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class DummyColor:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b


class DummyPose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class DummyMarkerRmv:
    def __init__(self, modified_pose, points, scale, color):
        self.modified_pose = modified_pose
        self.points = points
        self.scale = scale
        self.color = color


@pytest.fixture
def camera_manager():
    params = VisualizationParameters()
    params.setResolution(width=640, height=480, fov=60)
    params.updateCameraPosition(x=0, y=0, z=5, theta=0)
    return CameraManager(params)


@pytest.fixture
def blank_image():
    return np.ones((480, 640, 3), dtype=np.uint8) * 255


@pytest.fixture
def dummy_marker_visible():
    return DummyMarkerRmv(
        modified_pose=DummyPose(
            position=DummyPosition(0, 0, 0),
            orientation=DummyOrientation(0, 0, 0, 1),
        ),
        points=[DummyPosition(0, 0, 0), DummyPosition(1, 0, 0), DummyPosition(1, 1, 0)],
        scale=DummyScale(1, 1, 1),
        color=DummyColor(1, 0, 0),
    )


@pytest.fixture
def dummy_marker_behind_camera():
    return DummyMarkerRmv(
        modified_pose=DummyPose(
            position=DummyPosition(0, 0, 10),
            orientation=DummyOrientation(0, 0, 0, 1),
        ),
        points=[DummyPosition(0, 0, 0), DummyPosition(1, 0, 0)],
        scale=DummyScale(1, 1, 1),
        color=DummyColor(0, 1, 0),
    )


@pytest.fixture
def dummy_marker_not_enough_points():
    """Un LineStrip avec un seul point (pas de ligne possible)"""
    return DummyMarkerRmv(
        modified_pose=DummyPose(
            position=DummyPosition(0, 0, 5),
            orientation=DummyOrientation(0, 0, 0, 1),
        ),
        points=[DummyPosition(0, 0, 0)],
        scale=DummyScale(1, 1, 1),
        color=DummyColor(1, 0, 0),
    )


@pytest.mark.timeout(5)
def testDrawLineStripVisible(blank_image, camera_manager, dummy_marker_visible):
    LineStripDrawer.drawLineStrip(blank_image, dummy_marker_visible, camera_manager)

    assert np.any(np.all(blank_image == [0, 0, 255], axis=-1))


@pytest.mark.timeout(5)
def testDrawLineStripBehindCamera(
    blank_image, camera_manager, dummy_marker_behind_camera
):
    LineStripDrawer.drawLineStrip(
        blank_image, dummy_marker_behind_camera, camera_manager
    )

    assert not np.any(np.all(blank_image == [0, 255, 0], axis=-1))


@pytest.mark.timeout(5)
def testDrawLineStripNotEnoughPoints(
    blank_image, camera_manager, dummy_marker_not_enough_points
):
    LineStripDrawer.drawLineStrip(
        blank_image, dummy_marker_not_enough_points, camera_manager
    )

    assert not np.any(np.all(blank_image == [0, 0, 255], axis=-1))
