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

from ..camera import CameraManager


class GridDrawer:
    @staticmethod
    def drawGrid(
        image: np.ndarray,
        camera_manager: CameraManager,
        spacing_m: float = 1.0,
        color=(255, 255, 255),
    ):
        height, width = image.shape[:2]
        thickness = max(1, int(width / camera_manager.fx))

        max_x_m = camera_manager.camera_distance * np.tan(camera_manager.fov / 2)
        max_y_m = max_x_m * (height / width)
        if spacing_m <= 0:
            return
        x = 0
        # TODO: Using a while loop can be a source of performance issues, consider using a for loop instead.
        # Or evaluate the number of iterations and use a for loop with a range.

        while x < max_x_m:
            for sign in [-1, 1]:
                x_m = x * sign
                top_world = np.array([x_m, max_y_m, 0])
                bottom_world = np.array([x_m, -max_y_m, 0])

                top_img = camera_manager.projectToImage(
                    camera_manager.worldToCamera(top_world)
                )
                bottom_img = camera_manager.projectToImage(
                    camera_manager.worldToCamera(bottom_world)
                )

                if top_img and bottom_img:
                    cv2.line(image, top_img, bottom_img, color, thickness, cv2.LINE_AA)

            x += spacing_m

        y = 0
        while y < max_y_m:
            for sign in [-1, 1]:
                y_m = y * sign
                left_world = np.array([-max_x_m, y_m, 0])
                right_world = np.array([max_x_m, y_m, 0])

                left_img = camera_manager.projectToImage(
                    camera_manager.worldToCamera(left_world)
                )
                right_img = camera_manager.projectToImage(
                    camera_manager.worldToCamera(right_world)
                )

                if left_img and right_img:
                    cv2.line(image, left_img, right_img, color, thickness, cv2.LINE_AA)

            y += spacing_m
