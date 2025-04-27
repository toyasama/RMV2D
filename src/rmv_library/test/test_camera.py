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


def testExtrinsicMatrix(mockParams):
    extrinsics = CameraExtrinsics(mockParams)
    matrix = extrinsics.extrinsic_matrix

    assert matrix.shape == (4, 4)

    np.testing.assert_almost_equal(matrix[:3, 3], np.array([1.0, 2.0, 3.0]))


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


def testWorldToCamera(mockParams):
    manager = CameraManager(mockParams)

    world_point = np.array([1, 0, 0])
    camera_point = manager.worldToCamera(world_point)

    assert camera_point.shape == (3,)


def testProjectToImagePointInFront(mockParams):
    manager = CameraManager(mockParams)

    point_camera = np.array([0.5, 0.5, 2.0])
    pixel = manager.projectToImage(point_camera)

    assert pixel is not None
    x, y = pixel
    assert isinstance(x, int)
    assert isinstance(y, int)


def testProjectToImagePointBehind(mockParams, capsys):
    manager = CameraManager(mockParams)

    point_camera = np.array([0.5, 0.5, -1.0])
    pixel = manager.projectToImage(point_camera)

    assert pixel is None

    captured = capsys.readouterr()
    assert "Point is behind the camera." in captured.out


def testFx(mockParams):
    manager = CameraManager(mockParams)

    width, height, fov_deg = mockParams.resolution
    fov = np.deg2rad(fov_deg)
    fx_expected = width / (2 * np.tan(fov / 2))

    assert manager.fx == pytest.approx(fx_expected)


def testFov(mockParams):
    manager = CameraManager(mockParams)
    assert manager.fov == pytest.approx(np.deg2rad(mockParams.resolution[2]))


def testCameraDistance(mockParams):
    manager = CameraManager(mockParams)
    assert manager.camera_distance == pytest.approx(3.0)
