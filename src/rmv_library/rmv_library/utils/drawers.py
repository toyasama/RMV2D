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


import cv2
import numpy as np
from visualization_msgs.msg import Marker

from ..parameters.params import RmvParameters
from ..tf_management.transform_rmv import TransformDrawerInfo
from . import drawers_tools
from .camera import CameraManager
from .frame_position import FramesPosition


class DrawFrame:

    @staticmethod
    def drawMainFrame(
        camera_manager: CameraManager,
        image: np.ndarray,
        main_tf: str,
        parameters: RmvParameters,
        thickness: int,
    ):

        frame_pos = np.array([0, 0, 0])
        frame_pos_x_end = frame_pos + np.array([parameters.frames.axes_length, 0, 0])
        frame_pos_y_end = frame_pos + np.array([0, parameters.frames.axes_length, 0])

        proj_x_end = camera_manager.projectToImage(
            camera_manager.worldToCamera(frame_pos_x_end)
        )
        proj_y_end = camera_manager.projectToImage(
            camera_manager.worldToCamera(frame_pos_y_end)
        )
        proj = camera_manager.projectToImage(camera_manager.worldToCamera(frame_pos))
        frames_position = FramesPosition(main_tf, proj, proj_x_end, proj_y_end, 1)
        if proj != None:
            DrawFrame.drawFrame(image, frames_position, parameters, thickness)

    @staticmethod
    def projectAndDrawFrame(
        camera_manager: CameraManager,
        image: np.ndarray,
        transform_info: TransformDrawerInfo,
        parameters: RmvParameters,
        thickness: int,
    ) -> None:
        """Projects a 3D point to the image plane using the intrinsic matrix.

        Args:
            point: The 3D point to project.

        Returns:
            The 2D point on the image plane.

        """
        quat_array = np.array(
            [
                transform_info.pose_in_main_frame.rotation.w,
                transform_info.pose_in_main_frame.rotation.x,
                transform_info.pose_in_main_frame.rotation.y,
                transform_info.pose_in_main_frame.rotation.z,
            ]
        )

        # rot_mat = quat.quat2mat(quat_array)
        # TODO: check if the rotation is correct

        frame_pos = np.array(
            [
                transform_info.pose_in_main_frame.translation.x,
                transform_info.pose_in_main_frame.translation.y,
                transform_info.pose_in_main_frame.translation.z,
            ]
        )

        frame_pos_x_end = frame_pos + np.array([parameters.frames.axes_length, 0, 0])
        frame_pos_y_end = frame_pos + np.array([0, parameters.frames.axes_length, 0])

        proj_x_end = camera_manager.projectToImage(
            camera_manager.worldToCamera(frame_pos_x_end)
        )
        proj_y_end = camera_manager.projectToImage(
            camera_manager.worldToCamera(frame_pos_y_end)
        )
        proj = camera_manager.projectToImage(camera_manager.worldToCamera(frame_pos))

        if proj == None:
            return
        show_one_end = (
            transform_info.transform_name in parameters.frames.sub_frames
            or transform_info.transform_name in parameters.frames.main_frame
        )
        show_other_end = (
            transform_info.parent in parameters.frames.sub_frames
            or transform_info.parent in parameters.frames.main_frame
        )

        if parameters.frames.show_connections and show_one_end and show_other_end:
            start_connection = np.array(
                [
                    transform_info.start_connection.translation.x,
                    transform_info.start_connection.translation.y,
                    transform_info.start_connection.translation.z,
                ]
            )
            end_connection = np.array(
                [
                    transform_info.end_connection.translation.x,
                    transform_info.end_connection.translation.y,
                    transform_info.end_connection.translation.z,
                ]
            )

            proj_start_connection = camera_manager.projectToImage(
                camera_manager.worldToCamera(start_connection)
            )
            proj_end_connection = camera_manager.projectToImage(
                camera_manager.worldToCamera(end_connection)
            )
        else:
            proj_start_connection = None
            proj_end_connection = None

        frames_position = FramesPosition(
            transform_info.transform_name,
            proj,
            proj_x_end,
            proj_y_end,
            transform_info.opacity,
            proj_start_connection,
            proj_end_connection,
        )
        DrawFrame.drawFrame(image, frames_position, parameters, thickness)

    @staticmethod
    def drawFrame(
        image: np.ndarray,
        frames_position: FramesPosition,
        parameters: RmvParameters,
        thickness: int,
    ) -> None:
        """Draws a reference frame with a small circle and optional label."""
        if parameters.frames.show_axes:
            DrawFrame._drawAxes(
                image, frames_position, frames_position.opacity, thickness
            )  # TODO: fix text opacity
        if frames_position.name and parameters.frames.show_frame_names:
            DrawFrame.drawText(
                image,
                frames_position.name,
                (frames_position.start[0] + 10, frames_position.start[1] + 10),
                opacity=frames_position.opacity,
                thickness=thickness,
            )

        if (
            frames_position.start_connection != None
            and frames_position.end_connection != None
        ):
            DrawFrame.drawArrow(
                image,
                frames_position.start_connection,
                frames_position.end_connection,
                color=(150, 150, 150),
                opacity=frames_position.opacity,
                thickness=thickness,
            )

    @staticmethod
    def drawText(
        image: np.ndarray,
        text: str,
        position: tuple[int, int],
        font_scale: float = 0.3,
        color: tuple[int, int, int] = (255, 255, 255),
        thickness: int = 1,
        opacity=1,
    ) -> None:

        text_size, _ = cv2.getTextSize(
            text,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=font_scale,
            thickness=thickness,
        )
        text_width, text_height = text_size

        x, y = position
        if x + text_width > image.shape[1]:
            x = image.shape[1] - text_width
        if y - text_height < 0:
            y = text_height

        if opacity < 1.0:
            overlay = image.copy()
            cv2.putText(
                image,
                text,
                (x, y),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=font_scale,
                color=color,
                thickness=thickness,
                lineType=cv2.LINE_AA,
            )
            cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
        else:
            cv2.putText(
                image,
                text,
                (x, y),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=font_scale,
                color=color,
                thickness=thickness,
                lineType=cv2.LINE_AA,
            )

    @staticmethod
    def drawGrid(
        image: np.ndarray,
        spacing: int = 10,
        color: tuple[int, int, int] = (255, 255, 255),
        thickness: int = 1,
    ):
        """Draws a grid on the image with the specified spacing, color, and line thickness."""
        height, width = image.shape[:2]
        center_x, center_y = int(width // 2), int(height // 2)

        for x in range(center_x, width, spacing):
            cv2.line(image, (x, 0), (x, height), color, thickness)
        for x in range(center_x, 0, -spacing):
            cv2.line(image, (x, 0), (x, height), color, thickness)

        for y in range(center_y, height, spacing):
            cv2.line(image, (0, y), (width, y), color, thickness)
        for y in range(center_y, 0, -spacing):
            cv2.line(image, (0, y), (width, y), color, thickness)

    def _drawAxes(
        image: np.ndarray,
        frame_position: FramesPosition,
        opacity: float,
        thickness: int,
    ) -> None:
        """Draw X and Y axes of a frame."""
        DrawFrame.drawArrow(
            image,
            frame_position.start,
            frame_position.end_x,
            color=(0, 0, 255),
            opacity=opacity,
            thickness=thickness,
        )
        DrawFrame.drawArrow(
            image,
            frame_position.start,
            frame_position.end_y,
            color=(0, 255, 0),
            opacity=opacity,
            thickness=thickness,
        )

    def drawArrow(
        image: np.ndarray,
        start: tuple[int, int],
        end: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int = 1,
        tip_length: float = 0.1,
        opacity: float = 1.0,
    ) -> None:
        """Draw an arrow with optional opacity and ROI."""
        if opacity < 1.0 and opacity > 0:
            overlay = image.copy()
            cv2.arrowedLine(
                overlay,
                start,
                end,
                color=color,
                thickness=thickness,
                tipLength=tip_length,
            )
            cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
        elif opacity == 1.0:
            cv2.arrowedLine(
                image,
                start,
                end,
                color=color,
                thickness=thickness,
                tipLength=tip_length,
            )
        else:
            print("Invalid opacity value. Must be between 0 and 1.")


class DrawMarkers:

    @staticmethod
    def drawMarkers(
        image: np.ndarray, markers: list, camera_manager: CameraManager
    ) -> None:
        for marker in markers:
            match marker.type:
                case Marker.CUBE:
                    DrawMarkers.drawCube(image, marker, camera_manager)

                case Marker.SPHERE:
                    DrawMarkers.drawSphere(image, marker, camera_manager)

                case Marker.CYLINDER:
                    DrawMarkers.drawCylinder(image, marker, camera_manager)

                case Marker.LINE_STRIP:
                    DrawMarkers.drawLineStrip(image, marker, camera_manager)

    @staticmethod
    def drawCube(image: np.ndarray, marker, camera_manager: CameraManager) -> None:
        transformed_corners = drawers_tools.CubeTransformer.getTransformedCorners(
            marker
        )
        visible_corners = drawers_tools.CubeTransformer.getVisibleCorners(
            transformed_corners, camera_manager
        )
        if len(visible_corners) >= 4:
            drawers_tools.CubeDrawer.drawFaces(
                image,
                visible_corners,
                (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255),
            )
            drawers_tools.CubeDrawer.drawEdges(image, visible_corners, (0, 0, 0))

    @staticmethod
    def drawSphere(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.SphereDrawer.drawSphere(image, marker, camera_manager)

    @staticmethod
    def drawCylinder(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.CylinderDrawer.drawCylinder(image, marker, camera_manager)

    @staticmethod
    def drawArrow(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.ArrowDrawer.drawArrow(image, marker, camera_manager)

    def drawLineStrip(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.LineStripDrawer.drawLineStrip(image, marker, camera_manager)
