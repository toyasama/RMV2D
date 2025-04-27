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


def testDrawGridWithDefaultSpacingAndColor(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager)

    assert image.shape == (800, 600, 3)
    assert np.any(image)


def testDrawGridWithCustomSpacingAndColor(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager, spacing_m=2.0, color=(0, 255, 0))

    assert image.shape == (800, 600, 3)
    assert np.any(image)


def testDrawGridWithZeroSpacing(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager, spacing_m=0.0)
    assert image.shape == (800, 600, 3)
    assert not np.any(image)


def testDrawGridWithNegativeSpacing(camera_manager, empty_image):
    image = empty_image.copy()
    GridDrawer.drawGrid(image, camera_manager, spacing_m=-1.0)
    assert image.shape == (800, 600, 3)
    assert not np.any(image)
