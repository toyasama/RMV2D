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

import pytest
import numpy as np
from unittest.mock import MagicMock, patch

from rmv_visualization.visualization import Visualization
from rmv_library import (
    MarkerRmv,
    VisualizationParameters,
    FramesParameters,
    TransformGraph,
    RmvTransform,
)
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped


@pytest.fixture(autouse=True)
def patch_cv_bridge():
    with patch("rmv_visualization.visualization.CvBridge") as mock_bridge:
        yield mock_bridge


@pytest.fixture
def mockNode():
    node = MagicMock()
    node.create_publisher.return_value = MagicMock()
    node.get_logger.return_value.info = MagicMock()
    return node


@pytest.fixture
def mockParams():
    vis_params = VisualizationParameters()
    vis_params.setResolution(width=640, height=480, fov=90)
    vis_params.draw_grid = False
    vis_params.updateBackgroundColor(r=0, g=0, b=0)
    vis_params.updateGridColor(r=255, g=255, b=255)
    vis_params.updateCameraPosition(x=0.0, y=0.0, z=5.0, theta=0.0)
    vis_params.updateGridSpacing(0.5)
    frame_params = FramesParameters()
    frame_params.main_frame = "world"
    return MagicMock(visualization=vis_params, frames=frame_params)


@pytest.mark.timeout(5)
def testInitialization(mockNode, mockParams):
    vis = Visualization(mockNode, mockParams)
    assert vis.node == mockNode
    assert vis.rmv_params.visualization.width == 640
    assert vis.image.shape == (480, 640, 3)
    mockNode.get_logger().info.assert_called_with(
        "Visualization initialized successfully."
    )


@pytest.mark.timeout(5)
def testResetImageNoGrid(mockNode, mockParams):
    vis = Visualization(mockNode, mockParams)
    vis._image = None
    vis.resetImage()
    assert vis._image.shape == (480, 640, 3)
    assert np.all(vis._image == 0)


@pytest.mark.timeout(5)
def testImageProperty(mockNode, mockParams):
    vis = Visualization(mockNode, mockParams)
    img = vis.image
    assert img.shape == (480, 640, 3)
    assert img is not vis._image


@patch("rmv_visualization.visualization.DrawFrame")
@pytest.mark.timeout(5)
@patch("rmv_visualization.visualization.DrawMarkers")
def testUpdateImage(mockDrawMarkers, mockDrawFrame, mockNode, mockParams):

    vis = Visualization(mockNode, mockParams)
    tfGraph = TransformGraph(mockParams)
    tfGraph._main_frame = "world"
    tfGraph._graph.add_node("world")

    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = "world"
    transform_stamped.child_frame_id = "frame1"
    transform_stamped.transform.translation.x = 1.0
    transform_stamped.transform.translation.y = 2.0
    transform_stamped.transform.translation.z = 3.0
    transform_stamped.transform.rotation.w = 1.0

    rmv_transform = RmvTransform(transform_stamped, static=True)
    drawer_info = rmv_transform.drawer_info

    tfGraph.getTransformsFromMainFrame = MagicMock(
        return_value=[drawer_info, drawer_info]
    )

    markers = [MagicMock(spec=MarkerRmv)]

    vis.updateImage(markers, tfGraph)

    assert mockDrawFrame.drawMainFrame.called
    assert mockDrawFrame.projectAndDrawFrame.call_count == 2
    assert mockDrawMarkers.drawMarkers.called


@pytest.mark.timeout(5)
@patch("rmv_visualization.visualization.DrawFrame")
def testResetImageWithGrid(mockDrawFrame, mockNode, mockParams):
    mockParams.visualization.draw_grid = True
    vis = Visualization(mockNode, mockParams)
    vis.resetImage()
    assert mockDrawFrame.drawGrid.called


@pytest.mark.timeout(5)
def testVisualize(mockNode, mockParams):

    tfGraph = TransformGraph(mockParams)
    tfGraph._graph.add_node("world")

    vis = Visualization(mockNode, mockParams)
    vis.bridge.cv2_to_imgmsg.return_value = MagicMock(spec=Image)

    mockMarker = MagicMock(spec=MarkerRmv)

    vis.visualize([mockMarker], tfGraph)

    vis.publisher_raw.publish.assert_called()
    vis.bridge.cv2_to_imgmsg.assert_called()


@pytest.mark.timeout(5)
def testUpdateImageNoMainFrame(mockNode, mockParams):
    mockParams.frames.main_frame = None
    vis = Visualization(mockNode, mockParams)
    tfGraph = TransformGraph(mockParams)
    markers = [MagicMock(spec=MarkerRmv)]
    vis.updateImage(markers, tfGraph)
    assert np.all(vis._image == 0)
