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


from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


class MarkerRmvBase:

    def __init__(self, marker: Marker, reception_time: Time):
        self._marker = marker
        self._pub_time = reception_time
        self._modified_pose = None

    @property
    def identifier(self) -> tuple:
        return self._marker.ns, self._marker.id

    @property
    def pose(self) -> Pose:
        return self._marker.pose

    @property
    def modified_pose(self) -> Pose:
        return self._modified_pose if self._modified_pose else self.pose

    @modified_pose.setter
    def modified_pose(self, new_pose: Pose) -> None:
        self._modified_pose = new_pose

    @property
    def scale(self):
        return self._marker.scale

    @property
    def color(self):
        return self._marker.color

    @property
    def lifetime(self):
        return self._marker.lifetime

    @property
    def frame_id(self) -> str:
        return self._marker.header.frame_id

    @frame_id.setter
    def frame_id(self, frame_id: str):
        self._marker.header.frame_id = frame_id

    @property
    def points(self):
        return self._marker.points

    @property
    def type(self):
        return self._marker.type

    def getTransform(self):
        return self._marker.pose

    def isExpired(self, current_time: Time) -> bool:
        expiration_time = (
            self.lifetime.sec
            + (self.lifetime.nanosec * 1e-9)
            + self._pub_time.sec
            + (self._pub_time.nanosec * 1e-9)
        )
        current_time_in_seconds = current_time.sec + (current_time.nanosec * 1e-9)
        return current_time_in_seconds > expiration_time


class MarkerRmv(MarkerRmvBase):

    def __init__(self, marker: Marker, current_time: Time):
        super().__init__(marker, current_time)

    def __eq__(self, value):
        if not isinstance(value, MarkerRmv):
            return False
        return self.identifier == value.identifier
