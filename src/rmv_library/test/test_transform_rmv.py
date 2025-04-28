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
import pytest
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from rmv_library import (
    TransformDrawerInfo,
    RmvTransform,
)
from rmv_library.tf_management.transform_rmv import TransformBase


def makeTransformStamped(parent="world", child="camera", x=1.0):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation = Vector3(x=x, y=0.0, z=0.0)
    t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return t


def makeSampleTransform(x=1.0, y=2.0, z=3.0, qw=1.0):
    t = Transform()
    t.translation = Vector3(x=x, y=y, z=z)
    t.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=qw)
    return t


@pytest.mark.timeout(5)
def testTransformBaseDynamicExpiration():
    tf = makeTransformStamped()
    transform = TransformBase(tf, static=False)

    assert transform.opacity == pytest.approx(1.0, abs=0.05)
    assert transform.isExpired is False

    time.sleep(3.5)

    assert transform.opacity == 0.0
    assert transform.isExpired is True


@pytest.mark.timeout(5)
def testTransformBaseStaticNeverExpires():
    tf = makeTransformStamped()
    transform = TransformBase(tf, static=True)

    time.sleep(4)
    assert transform.opacity == 1.0
    assert transform.isExpired is False


@pytest.mark.timeout(5)
def testTransformBaseUpdateValid():
    tf = makeTransformStamped()
    transform = TransformBase(tf)

    time.sleep(1)
    updated_tf = makeTransformStamped(x=9.0)
    transform.update(updated_tf)

    assert transform.transform.translation.x == 9.0
    assert transform.opacity > 0.9


@pytest.mark.timeout(5)
def testTransformBaseUpdateInvalid(capfd):
    tf = makeTransformStamped("world", "camera")
    transform = TransformBase(tf)

    wrong_tf = makeTransformStamped("wrong_parent", "wrong_child")
    transform.update(wrong_tf)

    out, _ = capfd.readouterr()
    assert "Error" in out
    assert transform.transform.translation.x == 1.0


@pytest.mark.timeout(5)
def testTransformBaseEquality():
    tf1 = makeTransformStamped("world", "camera")
    tf2 = makeTransformStamped("world", "camera")
    tf3 = makeTransformStamped("map", "lidar")

    t1 = TransformBase(tf1)
    t2 = TransformBase(tf2)
    t3 = TransformBase(tf3)

    assert t1 == t2
    assert t1 != t3
    assert t1 == tf1
    assert t1 != tf3


@pytest.mark.timeout(5)
def testDrawerInfoToDraw():
    tf = makeTransformStamped()
    transform = TransformBase(tf)
    drawer_info = TransformDrawerInfo(transform)

    assert drawer_info.toDraw("world") is False

    drawer_info.update("world", Transform(), Transform(), Transform())
    assert drawer_info.toDraw("world") is True

    time.sleep(3.1)
    assert drawer_info.toDraw("world") is False


@pytest.mark.timeout(5)
def testRmvTransformIntegration():
    tf = makeTransformStamped()
    rmv = RmvTransform(tf, static=True)

    assert isinstance(rmv.drawer_info, TransformDrawerInfo)

    pose = makeSampleTransform(10.0, 0.0, 0.0)
    start_conn = makeSampleTransform(0.0, 1.0, 0.0)
    end_conn = makeSampleTransform(0.0, 0.0, 1.0)

    rmv.updateTransformDrawerInfo("world", pose, start_conn, end_conn)

    drawer = rmv.drawer_info
    assert drawer.main_frame == "world"

    assert drawer.pose_in_main_frame.translation.x == 10.0
    assert drawer.start_connection.translation.y == 1.0
    assert drawer.end_connection.translation.z == 1.0

    assert drawer.transform_name == tf.child_frame_id
    assert drawer.parent == tf.header.frame_id
    assert drawer.opacity == 1.0
