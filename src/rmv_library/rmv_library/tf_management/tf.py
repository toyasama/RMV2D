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


from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from ..tf_management.graph import TransformGraph
from ..parameters.params import RmvParameters

qos_profile_transient = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class TFManager:
    def __init__(self, node: Node, parameters: RmvParameters) -> None:
        super().__init__()
        self.node = node

        self._transform_graph = TransformGraph(parameters)

        self.node.create_subscription(TFMessage, "/tf", self.tfCallback, 10)
        self.node.create_subscription(
            TFMessage, "/tf_static", self.tfStaticCallback, qos_profile_transient
        )

        self.node.get_logger().info("TFManager initialized successfully.")

    def __del__(self):
        self.transform_graph.__del__()

    @property
    def transform_graph(self) -> TransformGraph:
        return self._transform_graph

    def tfCallback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.transform_graph.addTransform(transform, static=False)

    def tfStaticCallback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.transform_graph.addTransform(transform, static=True)
