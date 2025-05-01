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

import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from rmv_library import (
    MarkerRmv,
    MarkersHandler,
    RmvParameters,
    TFManager,
    TopicManager,
    TransformDrawerInfo,
    TransformUtils,
)
from rmv_visualization.visualization import Visualization
from rmv_msgs.srv import UpdateParams


class RMVChoreNode(Node):
    def __init__(self, config_name="params.yml"):
        super().__init__("rmv_chore", namespace="rmv")
        paremeter_file = self.getParametersFile(config_name)
        self.parameters: RmvParameters = RmvParameters(str(paremeter_file))
        self.tf_manager = TFManager(self, self.parameters)
        self.markers_handler = MarkersHandler()
        self.destroyed = False
        self.visualization = Visualization(self, self.parameters)
        self.topic_manager = TopicManager(self, self.markers_handler)
        self.period = 1 / self.parameters.visualization.fps
        self.create_timer(self.period, self.visualize)
        self.declare_service()
        self.get_logger().info("RMV Chore node initialized successfully.")

    def declare_service(self):
        self.srv = self.create_service(
            UpdateParams, "update_params", self._updateParamsSrv
        )

    def _updateParamsSrv(self, req: UpdateParams.Request, rep: UpdateParams.Response):
        success = True
        try:
            for key, value in req.params.items():
                try:
                    self.parameters.updateByKeyPath(key, value)
                except Exception as e:
                    success = False
                    rep._message = f"Error updating parameter {key}: {e}"
            rep.success = success
            return rep
        except Exception as e:
            self.get_logger().error(f"Error updating parameters: {e}")
            rep.message = f"Error updating parameters: {e}"
            return rep

    def getParametersFile(self, config_name):
        return os.path.join(
            get_package_share_directory("rmv_chore"), "config", config_name
        )

    def stop(self):
        if self.destroyed:
            return
        self.get_logger().info("Destroying RMV Chore node...")
        self.tf_manager.stop()
        self.markers_handler.stop()
        self.get_logger().info("RMV Chore node destroyed successfully.")
        self.destroyed = True

    def projectToMainFrame(
        self,
        markers: list[MarkerRmv],
        transforms: list[TransformDrawerInfo],
    ) -> list[MarkerRmv]:
        main_frame = self.tf_manager.transform_graph.main_frame
        transform_dict: dict[str, TransformDrawerInfo] = {
            transform.transform_name: transform.pose_in_main_frame
            for transform in transforms
        }
        projected_markers = []
        for marker in markers:
            if marker.frame_id == main_frame and main_frame is not None:
                marker.modified_pose = marker.pose
                projected_markers.append(marker)
            elif marker.frame_id in transform_dict:
                pose_in_main_frame = TransformUtils.transformPoseToParentFrame(
                    marker.pose,
                    transform_dict[marker.frame_id],
                )
                if pose_in_main_frame:
                    marker.modified_pose = pose_in_main_frame
                    projected_markers.append(marker)
        return projected_markers

    def visualize(self):
        markers = self.markers_handler.markers
        markers = self.projectToMainFrame(
            markers,
            self.tf_manager.transform_graph.getTransformsFromMainFrame(),
        )
        self.visualization.visualize(markers, self.tf_manager.transform_graph)
