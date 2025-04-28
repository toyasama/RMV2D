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
from unittest.mock import MagicMock
from rmv_library.utils.drawers_tools import GridDrawer
from rmv_library.utils.camera import CameraManager


@pytest.fixture
def camera_manager():
    cm = MagicMock(CameraManager)
    cm.fx = 1
    cm.fov = np.radians(60)
    cm.camera_distance = 10
    cm.worldToCamera.return_value = np.array([0, 0, 0])
    cm.projectToImage.return_value = (100, 100)
    return cm


@pytest.fixture
def empty_image():
    return np.zeros((800, 600, 3), dtype=np.uint8)


@pytest.mark.timeout(5)
def testDrawGridWithDefaultSpacingAndColor(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager)

    assert image.shape == (800, 600, 3)
    assert np.any(image)


@pytest.mark.timeout(5)
def testDrawGridWithCustomSpacingAndColor(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager, spacing_m=2.0, color=(0, 255, 0))

    assert image.shape == (800, 600, 3)
    assert np.any(image)


@pytest.mark.timeout(5)
def testDrawGridWithZeroSpacing(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager, spacing_m=0.0)
    assert image.shape == (800, 600, 3)
    assert not np.any(image)


@pytest.mark.timeout(5)
def testDrawGridWithNegativeSpacing(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager, spacing_m=-1.0)
    assert image.shape == (800, 600, 3)
    assert not np.any(image)
