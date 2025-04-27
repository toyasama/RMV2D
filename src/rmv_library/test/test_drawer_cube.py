import pytest
import numpy as np
import cv2

from rmv_library.utils.drawers_tools import (
    CubeTransformer,
    CubeDrawer,
)

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
    def __init__(self, modified_pose, scale):
        self.modified_pose = modified_pose
        self.scale = scale


class MockScale:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class MockCameraManager:
    def worldToCamera(self, point: np.ndarray) -> np.ndarray:
        return np.array([point[0], point[1], point[2] + 10])

    def projectToImage(self, point_camera: np.ndarray) -> tuple[int, int] | None:
        if point_camera[2] <= 0:
            return None
        return int(point_camera[0] + 320), int(point_camera[1] + 240)


# ------------------ FIXTURES ------------------


@pytest.fixture
def sample_marker():
    position = MockPosition(0, 0, 0)
    orientation = MockOrientation(0, 0, 0, 1)
    pose = MockPose(position, orientation)
    scale = MockScale(2, 2, 2)
    return MockMarkerRmv(pose, scale)


@pytest.fixture
def rotated_marker():
    angle = np.pi / 2
    qz = np.sin(angle / 2)
    qw = np.cos(angle / 2)
    orientation = MockOrientation(0, 0, qz, qw)
    position = MockPosition(1, 2, 3)
    pose = MockPose(position, orientation)
    scale = MockScale(2, 4, 2)
    return MockMarkerRmv(pose, scale)


@pytest.fixture
def sample_camera_manager():
    return MockCameraManager()


@pytest.fixture
def blank_image():
    return np.zeros((480, 640, 3), dtype=np.uint8)


# ------------------ TESTS CubeTransformer ------------------


def testGetTransformedCorners(sample_marker):
    corners = CubeTransformer.getTransformedCorners(sample_marker)
    assert corners.shape == (8, 3)

    expected_half_size = 1.0
    np.testing.assert_allclose(
        np.max(np.abs(corners), axis=0), expected_half_size, atol=1e-6
    )


def testGetTransformedCornersWithRotation(rotated_marker):
    corners = CubeTransformer.getTransformedCorners(rotated_marker)
    assert corners.shape == (8, 3)

    center = np.mean(corners, axis=0)
    np.testing.assert_allclose(center, np.array([1, 2, 3]), atol=1e-6)


def testGetVisibleCorners(sample_camera_manager):
    transformed_corners = np.array(
        [
            [1, 2, 5],
            [2, 3, 5],
            [3, 4, 5],
            [4, 5, 5],
            [5, 6, 5],
            [6, 7, 5],
            [7, 8, 5],
            [8, 9, 5],
        ]
    )
    visible_corners = CubeTransformer.getVisibleCorners(
        transformed_corners, sample_camera_manager
    )
    assert isinstance(visible_corners, list)
    assert len(visible_corners) == 8
    assert all(isinstance(pt, tuple) and len(pt) == 2 for pt in visible_corners)


# ------------------ TESTS CubeDrawer ------------------


def testDrawFaces(blank_image):
    corners = [
        (100, 100),
        (200, 100),
        (200, 200),
        (100, 200),
        (120, 120),
        (220, 120),
        (220, 220),
        (120, 220),
    ]
    CubeDrawer.drawFaces(blank_image, corners, (0, 255, 0))

    assert np.any(np.all(blank_image == (0, 255, 0), axis=-1))


def testDrawEdges(blank_image):
    corners = [
        (100, 100),
        (200, 100),
        (200, 200),
        (100, 200),
        (120, 120),
        (220, 120),
        (220, 220),
        (120, 220),
    ]
    CubeDrawer.drawEdges(blank_image, corners, (255, 0, 0))

    assert np.any(np.all(blank_image == (255, 0, 0), axis=-1))
