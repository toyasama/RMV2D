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
from unittest.mock import MagicMock

from rmv_library.utils.camera import CameraExtrinsics, CameraIntrinsics, CameraManager


@pytest.fixture
def mockParams():
    params = MagicMock()
    params.camera_position = (1.0, 2.0, 3.0, 90.0)
    params.resolution = (640, 480, 90.0)
    return params


@pytest.mark.timeout(5)
def testExtrinsicMatrix(mockParams):
    extrinsics = CameraExtrinsics(mockParams)
    matrix = extrinsics.extrinsic_matrix

    assert matrix.shape == (4, 4)

    np.testing.assert_almost_equal(matrix[:3, 3], np.array([1.0, 2.0, 3.0]))


@pytest.mark.timeout(5)
def testIntrinsicMatrix(mockParams):
    intrinsics = CameraIntrinsics(mockParams)
    matrix = intrinsics.intrinsic_matrix

    assert matrix.shape == (3, 3)

    width, height, fov_deg = mockParams.resolution
    fov = np.deg2rad(fov_deg)
    fx = width / (2 * np.tan(fov / 2))
    fy = fx
    cx = width / 2
    cy = height / 2

    assert matrix[0, 0] == pytest.approx(fx)
    assert matrix[1, 1] == pytest.approx(fy)
    assert matrix[0, 2] == pytest.approx(cx)
    assert matrix[1, 2] == pytest.approx(cy)
    assert matrix[2, 2] == 1


@pytest.mark.timeout(5)
def testWorldToCamera(mockParams):
    manager = CameraManager(mockParams)

    world_point = np.array([1, 0, 0])
    camera_point = manager.worldToCamera(world_point)

    assert camera_point.shape == (3,)


@pytest.mark.timeout(5)
def testProjectToImagePointInFront(mockParams):
    manager = CameraManager(mockParams)

    point_camera = np.array([0.5, 0.5, 2.0])
    pixel = manager.projectToImage(point_camera)

    assert pixel is not None
    x, y = pixel
    assert isinstance(x, int)
    assert isinstance(y, int)


@pytest.mark.timeout(5)
def testProjectToImagePointBehind(mockParams, capsys):
    manager = CameraManager(mockParams)

    point_camera = np.array([0.5, 0.5, -1.0])
    pixel = manager.projectToImage(point_camera)

    assert pixel is None

    captured = capsys.readouterr()
    assert "Point is behind the camera." in captured.out


@pytest.mark.timeout(5)
def testFx(mockParams):
    manager = CameraManager(mockParams)

    width, height, fov_deg = mockParams.resolution
    fov = np.deg2rad(fov_deg)
    fx_expected = width / (2 * np.tan(fov / 2))

    assert manager.fx == pytest.approx(fx_expected)


@pytest.mark.timeout(5)
def testFov(mockParams):
    manager = CameraManager(mockParams)
    assert manager.fov == pytest.approx(np.deg2rad(mockParams.resolution[2]))


@pytest.mark.timeout(5)
def testCameraDistance(mockParams):
    manager = CameraManager(mockParams)
    assert manager.camera_distance == pytest.approx(3.0)
