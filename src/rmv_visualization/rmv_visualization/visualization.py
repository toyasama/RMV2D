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

import threading

import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

from rmv_library import (
    CameraManager,
    DrawFrame,
    DrawMarkers,
    MarkerRmv,
    RmvParameters,
    TransformDrawerInfo,
    TransformGraph,
)


class Visualization(CameraManager):
    def __init__(self, node: Node, params: RmvParameters):
        """Initializes the visualization object, inheriting from CameraManager, with an option to draw a grid."""
        super().__init__(params.visualization)
        self.image_lock = threading.RLock()
        self.rmv_params = params
        self.node = node
        self.bridge = CvBridge()
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._image = self.resetImage()
        self.publisher_raw = self.node.create_publisher(Image, "image", qos)
        self.node.get_logger().info("Visualization initialized successfully.")

    @property
    def image(self):
        with self.image_lock:
            if self._image is None:
                return None
            return self._image.copy()

    def updateImage(self, markers: list[MarkerRmv], transform_graph: TransformGraph):
        """Generates an image centered on `main_tf` with a grid in the background."""
        self.resetImage()
        main_frame = transform_graph.main_frame
        if not main_frame:
            return
        axe_length_px = int(
            self.rmv_params.frames.axes_length * 0.01 * self.fx / self.camera_distance
        )
        axes_thickness = max(1, axe_length_px)
        DrawFrame.drawMainFrame(
            self, self._image, main_frame, self.rmv_params, axes_thickness
        )
        tf_drawer_info: list[TransformDrawerInfo] = (
            transform_graph.getTransformsFromMainFrame()
        )

        for frame in tf_drawer_info:
            if (
                self.rmv_params.frames.show_sub_frames
                or frame.transform_name in self.rmv_params.frames.sub_frames
            ):
                DrawFrame.projectAndDrawFrame(
                    self, self._image, frame, self.rmv_params, axes_thickness
                )
        DrawMarkers.drawMarkers(self._image, markers, self)

    def resetImage(self):
        """Creates a new image with a black background."""
        background_color = (
            self.rmv_params.visualization.background_color["b"],
            self.rmv_params.visualization.background_color["g"],
            self.rmv_params.visualization.background_color["r"],
        )
        self._image = np.full(
            (
                self.rmv_params.visualization.height,
                self.rmv_params.visualization.width,
                3,
            ),
            background_color,
            dtype=np.uint8,
        )

        if self.rmv_params.visualization.draw_grid:
            spacing_in_px = max(
                1,
                int(
                    (self.rmv_params.visualization.grid_spacing / self.camera_distance)
                    * self.fx
                ),
            )
            grid_color = (
                self.rmv_params.visualization.grid_color["b"],
                self.rmv_params.visualization.grid_color["g"],
                self.rmv_params.visualization.grid_color["r"],
            )
            grid_thickness = max(1, int(0.01 * spacing_in_px))
            DrawFrame.drawGrid(
                self._image, spacing_in_px, color=grid_color, thickness=grid_thickness
            )
        return self._image

    def visualize(
        self, markers: list[MarkerRmv], transform_graph: TransformGraph
    ) -> None:
        image = None
        with self.image_lock:
            self.updateImage(markers, transform_graph)
            image = self._image.copy()

        if image is not None:
            image_raw = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.publisher_raw.publish(image_raw)
