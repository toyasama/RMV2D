import pytest
import numpy as np
from unittest.mock import Mock, patch
from visualization_msgs.msg import Marker

from rmv_library.utils.drawers import DrawFrame, DrawMarkers
from rmv_library.utils.drawers import FramesPosition
from rmv_library.utils.camera import CameraManager
from rmv_library.parameters.params import RmvParameters
from rmv_library.tf_management.transform_rmv import TransformDrawerInfo


@pytest.fixture
def mock_camera_manager():
    """Create a mock camera manager for testing."""
    camera_manager = Mock(spec=CameraManager)
    camera_manager.projectToImage.return_value = np.array([100, 100])
    camera_manager.worldToCamera.return_value = np.array([1, 1, 1])
    return camera_manager


@pytest.fixture
def mock_image():
    """Create a mock image for drawing operations."""
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def mock_parameters():
    """Create mock parameters for testing."""
    params = Mock(spec=RmvParameters)
    params.frames = Mock()
    params.frames.axes_length = 0.1
    params.frames.show_axes = True
    params.frames.show_frame_names = True
    params.frames.show_connections = True
    params.frames.sub_frames = ["base_link", "camera_link"]
    params.frames.main_frame = ["map"]
    return params


@pytest.fixture
def mock_transform_info():
    """Create a mock TransformDrawerInfo for testing."""
    transform_info = Mock(spec=TransformDrawerInfo)
    transform_info.transform_name = "base_link"
    transform_info.parent = "map"
    transform_info.opacity = 1.0

    # Create mock poses
    transform_info.pose_in_main_frame = Mock()
    transform_info.pose_in_main_frame.translation = Mock()
    transform_info.pose_in_main_frame.translation.x = 1.0
    transform_info.pose_in_main_frame.translation.y = 1.0
    transform_info.pose_in_main_frame.translation.z = 1.0

    transform_info.pose_in_main_frame.rotation = Mock()
    transform_info.pose_in_main_frame.rotation.w = 1.0
    transform_info.pose_in_main_frame.rotation.x = 0.0
    transform_info.pose_in_main_frame.rotation.y = 0.0
    transform_info.pose_in_main_frame.rotation.z = 0.0

    # Connection poses
    transform_info.start_connection = Mock()
    transform_info.start_connection.translation = Mock()
    transform_info.start_connection.translation.x = 0.0
    transform_info.start_connection.translation.y = 0.0
    transform_info.start_connection.translation.z = 0.0

    transform_info.end_connection = Mock()
    transform_info.end_connection.translation = Mock()
    transform_info.end_connection.translation.x = 1.0
    transform_info.end_connection.translation.y = 1.0
    transform_info.end_connection.translation.z = 1.0

    return transform_info


@pytest.fixture
def mock_frames_position():
    """Create a mock FramesPosition for testing."""
    return FramesPosition(
        name="test_frame",
        start=np.array([100, 100]),
        end_x=np.array([150, 100]),
        end_y=np.array([100, 150]),
        opacity=1.0,
        start_connection=np.array([80, 80]),
        end_connection=np.array([120, 120]),
    )


class TestDrawFrame:

    def test_draw_main_frame(self, mock_camera_manager, mock_image, mock_parameters):
        """Test if drawMainFrame correctly projects and draws the main frame."""
        with patch.object(DrawFrame, "drawFrame") as mock_draw_frame:
            DrawFrame.drawMainFrame(
                mock_camera_manager, mock_image, "map", mock_parameters, 2
            )

            assert mock_draw_frame.call_count == 1

            assert mock_camera_manager.projectToImage.call_count >= 3
            assert mock_camera_manager.worldToCamera.call_count >= 3

    def test_project_and_draw_frame(
        self, mock_camera_manager, mock_image, mock_transform_info, mock_parameters
    ):
        """Test if projectAndDrawFrame correctly projects and draws a frame."""
        with patch.object(DrawFrame, "drawFrame") as mock_draw_frame:
            result = DrawFrame.projectAndDrawFrame(
                mock_camera_manager, mock_image, mock_transform_info, mock_parameters, 2
            )
            assert mock_draw_frame.call_count == 1
            assert mock_camera_manager.projectToImage.call_count >= 3
            assert mock_camera_manager.worldToCamera.call_count >= 3

    def testProjectAndDrawFrameNotInView(
        self, mock_camera_manager, mock_image, mock_transform_info, mock_parameters
    ):
        """Test behavior when frame is not in view."""
        mock_camera_manager.projectToImage.return_value = np.array([])

        with patch.object(DrawFrame, "drawFrame") as mock_draw_frame, patch(
            "builtins.print"
        ) as mock_print:
            DrawFrame.projectAndDrawFrame(
                mock_camera_manager, mock_image, mock_transform_info, mock_parameters, 2
            )

            assert mock_draw_frame.call_count == 0

            mock_print.assert_called_once_with("Frame not in view")

    def testDrawFrameWithAxes(self, mock_image, mock_frames_position, mock_parameters):
        with patch.object(DrawFrame, "_drawAxes") as mock_draw_axes, patch.object(
            DrawFrame, "drawText"
        ) as mock_draw_text, patch.object(DrawFrame, "drawArrow") as mock_draw_arrow:

            DrawFrame.drawFrame(mock_image, mock_frames_position, mock_parameters, 2)

            assert mock_draw_axes.call_count == 1
            assert mock_draw_text.call_count == 1
            assert mock_draw_arrow.call_count == 1

    def testDrawFrameWithoutAxes(
        self, mock_image, mock_frames_position, mock_parameters
    ):
        mock_parameters.frames.show_axes = False

        with patch.object(DrawFrame, "_drawAxes") as mock_draw_axes, patch.object(
            DrawFrame, "drawText"
        ) as mock_draw_text, patch.object(DrawFrame, "drawArrow") as mock_draw_arrow:

            DrawFrame.drawFrame(mock_image, mock_frames_position, mock_parameters, 2)

            assert mock_draw_axes.call_count == 0
            assert mock_draw_text.call_count == 1
            assert mock_draw_arrow.call_count == 1

    def testDrawText(self, mock_image):
        with patch("cv2.putText") as mock_put_text, patch(
            "cv2.getTextSize", return_value=((50, 20), 0)
        ) as mock_get_text_size, patch("cv2.addWeighted") as mock_add_weighted:

            DrawFrame.drawText(mock_image, "Test Text", (100, 100), opacity=1.0)
            assert mock_put_text.call_count == 1
            assert mock_add_weighted.call_count == 0

            mock_put_text.reset_mock()
            mock_add_weighted.reset_mock()

            DrawFrame.drawText(mock_image, "Test Text", (100, 100), opacity=0.5)
            assert mock_put_text.call_count == 1
            assert mock_add_weighted.call_count == 1

    def testDrawGrid(self, mock_image):
        with patch("cv2.line") as mock_line:
            DrawFrame.drawGrid(mock_image, spacing=50)
            assert mock_line.call_count > 0

    def testDrawAxes(self, mock_image, mock_frames_position):
        with patch.object(DrawFrame, "drawArrow") as mock_draw_arrow:
            DrawFrame._drawAxes(mock_image, mock_frames_position, 1.0, 2)

            assert mock_draw_arrow.call_count == 2

            args_x = mock_draw_arrow.call_args_list[0][1]
            assert args_x["color"] == (0, 0, 255)

            args_y = mock_draw_arrow.call_args_list[1][1]
            assert args_y["color"] == (0, 255, 0)

    def testDrawArrow(self, mock_image):
        """Test the drawArrow method."""
        with patch("cv2.arrowedLine") as mock_arrowed_line, patch(
            "cv2.addWeighted"
        ) as mock_add_weighted, patch("builtins.print") as mock_print:

            DrawFrame.drawArrow(
                mock_image,
                (100, 100),
                (200, 200),
                (255, 0, 0),
                thickness=2,
                opacity=1.0,
            )
            assert mock_arrowed_line.call_count == 1
            assert mock_add_weighted.call_count == 0

            mock_arrowed_line.reset_mock()
            mock_add_weighted.reset_mock()

            DrawFrame.drawArrow(
                mock_image,
                (100, 100),
                (200, 200),
                (255, 0, 0),
                thickness=2,
                opacity=0.5,
            )
            assert mock_arrowed_line.call_count == 1
            assert mock_add_weighted.call_count == 1

            mock_arrowed_line.reset_mock()
            mock_add_weighted.reset_mock()

            DrawFrame.drawArrow(
                mock_image, (100, 100), (200, 200), (255, 0, 0), thickness=2, opacity=-1
            )
            assert mock_arrowed_line.call_count == 0
            assert mock_add_weighted.call_count == 0
            mock_print.assert_called_with(
                "Invalid opacity value. Must be between 0 and 1."
            )


class TestDrawMarkers:

    def testDrawMarkers(self, mock_image, mock_camera_manager):
        """Test if drawMarkers correctly delegates to the appropriate drawing methods."""
        cube_marker = Mock()
        cube_marker.type = Marker.CUBE

        sphere_marker = Mock()
        sphere_marker.type = Marker.SPHERE

        cylinder_marker = Mock()
        cylinder_marker.type = Marker.CYLINDER

        line_strip_marker = Mock()
        line_strip_marker.type = Marker.LINE_STRIP

        markers = [cube_marker, sphere_marker, cylinder_marker, line_strip_marker]

        with patch.object(DrawMarkers, "drawCube") as mock_draw_cube, patch.object(
            DrawMarkers, "drawSphere"
        ) as mock_draw_sphere, patch.object(
            DrawMarkers, "drawCylinder"
        ) as mock_draw_cylinder, patch.object(
            DrawMarkers, "drawLineStrip"
        ) as mock_draw_line_strip:

            DrawMarkers.drawMarkers(mock_image, markers, mock_camera_manager)

            mock_draw_cube.assert_called_once_with(
                mock_image, cube_marker, mock_camera_manager
            )
            mock_draw_sphere.assert_called_once_with(
                mock_image, sphere_marker, mock_camera_manager
            )
            mock_draw_cylinder.assert_called_once_with(
                mock_image, cylinder_marker, mock_camera_manager
            )
            mock_draw_line_strip.assert_called_once_with(
                mock_image, line_strip_marker, mock_camera_manager
            )

    def testDrawCube(self, mock_image, mock_camera_manager):
        """Test if drawCube correctly delegates to the CubeTransformer and CubeDrawer."""
        marker = Mock()
        marker.color = Mock()
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0

        with patch(
            "rmv_library.utils.drawers_tools.cube.CubeTransformer.getTransformedCorners",
            return_value=["corner1", "corner2"],
        ) as mock_get_transformed_corners, patch(
            "rmv_library.utils.drawers_tools.cube.CubeTransformer.getVisibleCorners",
            return_value=["corner1", "corner2", "corner3", "corner4"],
        ) as mock_get_visible_corners, patch(
            "rmv_library.utils.drawers_tools.cube.CubeDrawer.drawFaces"
        ) as mock_draw_faces, patch(
            "rmv_library.utils.drawers_tools.cube.CubeDrawer.drawEdges"
        ) as mock_draw_edges:

            DrawMarkers.drawCube(mock_image, marker, mock_camera_manager)

            mock_get_transformed_corners.assert_called_once_with(marker)
            mock_get_visible_corners.assert_called_once_with(
                ["corner1", "corner2"], mock_camera_manager
            )
            mock_draw_faces.assert_called_once_with(
                mock_image,
                ["corner1", "corner2", "corner3", "corner4"],
                (0.0 * 255, 0.5 * 255, 1.0 * 255),
            )
            mock_draw_edges.assert_called_once_with(
                mock_image, ["corner1", "corner2", "corner3", "corner4"], (0, 0, 0)
            )

    def testDrawSphere(self, mock_image, mock_camera_manager):
        """Test if drawSphere correctly delegates to the SphereDrawer."""
        marker = Mock()

        with patch(
            "rmv_library.utils.drawers_tools.sphere.SphereDrawer.drawSphere"
        ) as mock_draw_sphere:
            DrawMarkers.drawSphere(mock_image, marker, mock_camera_manager)
            mock_draw_sphere.assert_called_once_with(
                mock_image, marker, mock_camera_manager
            )

    def testDrawCylinder(self, mock_image, mock_camera_manager):
        """Test if drawCylinder correctly delegates to the CylinderDrawer."""
        marker = Mock()

        with patch(
            "rmv_library.utils.drawers_tools.cylinder.CylinderDrawer.drawCylinder"
        ) as mock_draw_cylinder:
            DrawMarkers.drawCylinder(mock_image, marker, mock_camera_manager)
            mock_draw_cylinder.assert_called_once_with(
                mock_image, marker, mock_camera_manager
            )

    def testDrawLineStrip(self, mock_image, mock_camera_manager):
        """Test if drawLineStrip correctly delegates to the LineStripDrawer."""
        marker = Mock()

        with patch(
            "rmv_library.utils.drawers_tools.line_strip.LineStripDrawer.drawLineStrip"
        ) as mock_draw_line_strip:
            DrawMarkers.drawLineStrip(mock_image, marker, mock_camera_manager)
            mock_draw_line_strip.assert_called_once_with(
                mock_image, marker, mock_camera_manager
            )
