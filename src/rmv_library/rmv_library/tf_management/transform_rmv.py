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

import time

from geometry_msgs.msg import Transform, TransformStamped


class TransformBase:
    """Class to manage a Transform.
       Use as wrapper for ros2 TransformStamped.
       Evaluate the lifetime of the transform and manage its pose.
    Args:
        transform_stamped (TransformStamped): The transform to manage.
        static (bool): Whether the transform is static or not.
    """

    __expiration_duration = 3.0

    def __init__(self, transform_stamped: TransformStamped, static: bool = False):
        self._name: str = transform_stamped.child_frame_id
        self._parent: str = transform_stamped.header.frame_id
        self._transform: Transform = transform_stamped.transform
        self._received_time: float = time.time()
        self._static = static

    def __eq__(self, value) -> bool:
        if isinstance(value, TransformBase):
            return self.name == value.name and self.parent == value.parent
        if isinstance(value, TransformStamped):
            return (
                self.name == value.child_frame_id
                and self.parent == value.header.frame_id
            )
        return False

    @property
    def name(self) -> str:
        return self._name

    @property
    def parent(self) -> str:
        return self._parent

    @property
    def transform(self) -> Transform:
        return self._transform

    @property
    def opacity(self) -> float:
        if self._static:
            return 1.0
        elapsed_time = time.time() - self._received_time
        return max(0, 1.0 - elapsed_time / self.__expiration_duration)

    @property
    def isExpired(self) -> bool:
        return (
            not self._static
            and time.time() > self._received_time + self.__expiration_duration
        )

    def update(self, transform_stamped: TransformStamped) -> None:
        """Update the transform with a new TransformStamped value if the child_id and frame_id match.
        Args:
            transform_stamped (TransformStamped): The new transform to update.
        """
        if not self == transform_stamped:
            print("Error: Trying to update a transform with different name or parent.")
            print(f"Old transform: {self.name} -> {self.parent}")
            print(
                f"New transform: {transform_stamped.child_frame_id} -> {transform_stamped.header.frame_id}"
            )
            return
        self._received_time = time.time()
        self._transform = transform_stamped.transform


class TransformDrawerInfo:
    """Class to manage the information to draw a transform.
       Allows to manage the pose of the transform in the main frame.
    Args:
        Transform_base (TransformBase): The transform to manage.
    """

    def __init__(self, Transform_base: TransformBase):
        self._main_frame: str = ""
        self._pose_in_main_frame: Transform = Transform()
        self._start_connection: Transform = Transform()
        self._end_connection: Transform = Transform()
        self.__associated_transform: TransformBase = Transform_base

    def __str__(self) -> str:
        return f"Main frame: {self._main_frame}, Start connection: {self._start_connection.translation}, End connection: {self._end_connection.translation}"

    def __repr__(self):
        return self.__str__()

    @property
    def transform_name(self) -> str:
        return self.__associated_transform.name

    @property
    def parent(self) -> str:
        return self.__associated_transform.parent

    @property
    def pose_in_main_frame(self) -> Transform:
        return self._pose_in_main_frame

    @property
    def main_frame(self) -> str:
        return self._main_frame

    @property
    def start_connection(self) -> Transform:
        return self._start_connection

    @property
    def end_connection(self) -> Transform:
        return self._end_connection

    @property
    def opacity(self) -> float:
        return self.__associated_transform.opacity

    def toDraw(self, main_frame) -> bool:
        """Check if the transform should be drawn in the main frame."""
        return (
            self._main_frame == main_frame
        ) and not self.__associated_transform.isExpired

    def update(
        self,
        main_frame: str,
        pose_in_main_frame: Transform,
        start_connection: Transform,
        end_connection: Transform,
    ) -> None:
        """Update drawing information"""
        self._main_frame = main_frame
        self._pose_in_main_frame = pose_in_main_frame
        self._start_connection = start_connection
        self._end_connection = end_connection


class RmvTransform(TransformBase):
    """Class to manage a Transform with additional information for visualization.
       Inherits from TransformBase and adds information for drawing the transform.
    Args:
        transform_stamped (TransformStamped): The transform to manage.
        static (bool): Whether the transform is static or not.
    """

    def __init__(self, transform_stamped: TransformStamped, static: bool):
        super().__init__(transform_stamped, static)
        self._drawer_info = TransformDrawerInfo(self)
        self._initial_direction = True

    def __eq__(self, value) -> bool:
        return super().__eq__(value)

    def __hash__(self) -> int:
        return hash((self.name, self.parent))

    @property
    def drawer_info(self) -> TransformDrawerInfo:
        return self._drawer_info

    def updateTransformDrawerInfo(
        self,
        main_frame: str,
        pose_in_main_frame: Transform,
        start_connection: Transform,
        end_connection: Transform,
    ) -> None:
        self._drawer_info.update(
            main_frame, pose_in_main_frame, start_connection, end_connection
        )
