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
import numpy as np

from rmv_library.utils.drawers_tools import CylinderDrawer

# ------------------ MOCKS ------------------


class MockPose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class MockPosition:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class MockOrientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class MockMarkerRmv:
    def __init__(self, modified_pose, scale, color):
        self.modified_pose = modified_pose
        self.scale = scale
        self.color = color


class MockScale:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class MockColor:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b


class MockCameraManager:
    def worldToCamera(self, world_point):
        return np.array([world_point[0], world_point[1], world_point[2] + 5])

    def projectToImage(self, camera_point):
        fx = fy = 50
        cx, cy = 320, 240
        x_img = int(cx + camera_point[0] * fx)
        y_img = int(cy - camera_point[1] * fy)
        return (x_img, y_img)


# ------------------ FIXTURES ------------------


@pytest.fixture
def sample_camera_manager():
    return MockCameraManager()


@pytest.fixture
def blank_image():
    return np.zeros((480, 640, 3), dtype=np.uint8)


# ------------------ TESTS CylinderDrawer ------------------


@pytest.mark.timeout(5)
def testDrawCylinderSimple(blank_image, sample_camera_manager):
    position = MockPosition(0, 0, 0)
    orientation = MockOrientation(0, 0, 0, 1)
    pose = MockPose(position, orientation)
    scale = MockScale(2, 1.0, 4)
    color = MockColor(1, 0, 0)
    marker = MockMarkerRmv(pose, scale, color)

    CylinderDrawer.drawCylinder(blank_image, marker, sample_camera_manager)

    assert np.any(blank_image[:, :, 2] > 0)


@pytest.mark.timeout(5)
def testDrawCylinderFlat(blank_image, sample_camera_manager):
    position = MockPosition(0, 0, 0)
    orientation = MockOrientation(0, 0, 0, 1)
    pose = MockPose(position, orientation)
    scale = MockScale(2, 0.5, 4)
    color = MockColor(0, 1, 0)
    marker = MockMarkerRmv(pose, scale, color)

    CylinderDrawer.drawCylinder(blank_image, marker, sample_camera_manager)

    assert np.any(blank_image[:, :, 1] > 0)


@pytest.mark.timeout(5)
def testDrawCylinderLarge(blank_image, sample_camera_manager):
    position = MockPosition(0, 0, 5)
    orientation = MockOrientation(0, 0, 0, 1)
    pose = MockPose(position, orientation)
    scale = MockScale(10, 10, 10)
    color = MockColor(0, 0, 1)
    marker = MockMarkerRmv(pose, scale, color)

    CylinderDrawer.drawCylinder(blank_image, marker, sample_camera_manager)

    assert np.any(blank_image[:, :, 0] > 0)


@pytest.mark.timeout(5)
def testDrawCylinderTiny(blank_image, sample_camera_manager):
    position = MockPosition(0, 0, 0)
    orientation = MockOrientation(0, 0, 0, 1)
    pose = MockPose(position, orientation)
    scale = MockScale(0.1, 0.1, 0.2)
    color = MockColor(1, 1, 0)
    marker = MockMarkerRmv(pose, scale, color)

    CylinderDrawer.drawCylinder(blank_image, marker, sample_camera_manager)

    assert np.any(blank_image[:, :, :2] > 0)


@pytest.mark.timeout(5)
def testDrawCylinderInvisible(blank_image, sample_camera_manager):
    position = MockPosition(0, 0, -20)
    orientation = MockOrientation(0, 0, 0, 1)
    pose = MockPose(position, orientation)
    scale = MockScale(2, 2, 4)
    color = MockColor(1, 0, 0)
    marker = MockMarkerRmv(pose, scale, color)

    CylinderDrawer.drawCylinder(blank_image, marker, sample_camera_manager)

    assert np.all(blank_image == 0)
