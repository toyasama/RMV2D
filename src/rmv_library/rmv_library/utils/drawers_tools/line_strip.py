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
import tf_transformations as tf

from ...markers_management.markers import MarkerRmv
from ..camera import CameraManager


class LineStripDrawer:
    @staticmethod
    def drawLineStrip(
        image: np.ndarray, marker: MarkerRmv, camera_manager: CameraManager
    ) -> None:
        if len(marker.points) < 2:
            return

        quaternion = [
            marker.modified_pose.orientation.x,
            marker.modified_pose.orientation.y,
            marker.modified_pose.orientation.z,
            marker.modified_pose.orientation.w,
        ]
        rotation_matrix = tf.quaternion_matrix(quaternion)[:3, :3]

        position = np.array(
            [
                marker.modified_pose.position.x,
                marker.modified_pose.position.y,
                marker.modified_pose.position.z,
            ]
        )

        transformed_points = [
            position + rotation_matrix @ np.array([p.x, p.y, p.z])
            for p in marker.points
        ]

        projected_points = []
        for p in transformed_points:
            camera_coords = camera_manager.worldToCamera(p)
            if camera_coords[2] > 0:  # Vérifier que le point est devant la caméra
                image_point = camera_manager.projectToImage(camera_coords)
                if image_point is not None:
                    projected_points.append(image_point)

        if len(projected_points) < 2:
            return

        fx = camera_manager.fx
        projected_thickness = min(
            1, int(marker.scale.x * fx / camera_manager.camera_distance)
        )

        for i in range(len(projected_points) - 1):
            color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)

            cv2.line(
                image,
                projected_points[i],
                projected_points[i + 1],
                color,
                projected_thickness,
            )
