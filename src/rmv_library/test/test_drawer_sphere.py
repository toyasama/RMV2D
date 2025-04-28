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
import cv2
import pytest

from rmv_library.utils.drawers_tools.sphere import SphereDrawer
from rmv_library.utils.camera import CameraManager
from rmv_library.parameters.params import VisualizationParameters

# --- MOCK SIMPLE CLASSES ---


class DummyPose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


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


class DummyColor:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b


class DummyScale:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class DummyMarkerRmv:
    def __init__(self, position, orientation, scale, color):
        self.modified_pose = DummyPose(position, orientation)
        self.scale = scale
        self.color = color


# --- FIXTURES ---


@pytest.fixture
def camera_manager():
    params = VisualizationParameters()
    params.setResolution(width=640, height=480, fov=60)
    params.updateCameraPosition(x=0, y=0, z=5, theta=0)
    return CameraManager(params)


@pytest.fixture
def blank_image():
    return np.ones((480, 640, 3), dtype=np.uint8) * 255


# --- TESTS ---


@pytest.mark.timeout(5)
def testDrawSimpleSphere(blank_image, camera_manager):
    marker = DummyMarkerRmv(
        position=DummyPosition(0, 0, 0),
        orientation=DummyOrientation(0, 0, 0, 1),
        scale=DummyScale(1, 1, 1),
        color=DummyColor(1, 0, 0),
    )

    SphereDrawer.drawSphere(blank_image, marker, camera_manager)

    assert np.any(np.all(blank_image == [0, 0, 255], axis=-1))


@pytest.mark.timeout(5)
def testDrawSphereBehindCamera(blank_image, camera_manager):
    marker = DummyMarkerRmv(
        position=DummyPosition(0, 0, 10),
        orientation=DummyOrientation(0, 0, 0, 1),
        scale=DummyScale(1, 1, 1),
        color=DummyColor(0, 1, 0),
    )

    SphereDrawer.drawSphere(blank_image, marker, camera_manager)
    assert not np.any(np.all(blank_image == [0, 255, 0], axis=-1))


@pytest.mark.timeout(5)
def testDrawEllipseNotCircular(blank_image, camera_manager):
    marker = DummyMarkerRmv(
        position=DummyPosition(0, 0, 0),
        orientation=DummyOrientation(0, 0, 0, 1),
        scale=DummyScale(2, 1, 1),
        color=DummyColor(0, 0, 1),
    )

    SphereDrawer.drawSphere(blank_image, marker, camera_manager)

    assert np.any(np.all(blank_image == [255, 0, 0], axis=-1))
