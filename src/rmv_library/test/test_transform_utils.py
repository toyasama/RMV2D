import pytest
import numpy as np
from geometry_msgs.msg import Transform, Pose
from rmv_library.tf_management.transform_utils import (
    TransformUtils,
)
import tf_transformations as tf

# ----------- Fixtures -----------


@pytest.fixture
def identity_transform():
    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = 0.0, 0.0, 0.0
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = 0.0, 0.0, 0.0, 1.0
    return t


@pytest.fixture
def simple_transform():
    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = 1.0, 2.0, 3.0
    quat = tf.quaternion_from_euler(0.1, 0.2, 0.3)
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = quat
    return t


@pytest.fixture
def simple_pose():
    p = Pose()
    p.position.x, p.position.y, p.position.z = 1.0, 2.0, 3.0
    quat = tf.quaternion_from_euler(0.1, 0.2, 0.3)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p


@pytest.fixture
def another_transform():
    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = -1.0, 0.5, 2.0
    quat = tf.quaternion_from_euler(-0.1, 0.2, -0.3)
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = quat
    return t


# ----------- Tests -----------
def testInvertTransformIdentity(identity_transform):
    inverted = TransformUtils.invertTransform(identity_transform)
    assert np.allclose(
        [inverted.translation.x, inverted.translation.y, inverted.translation.z],
        [0, 0, 0],
    )
    assert np.allclose(
        [
            inverted.rotation.x,
            inverted.rotation.y,
            inverted.rotation.z,
            inverted.rotation.w,
        ],
        [0, 0, 0, 1],
    )


def testInvertTransformSimple(simple_transform):
    inverted = TransformUtils.invertTransform(simple_transform)
    double_inverted = TransformUtils.invertTransform(inverted)
    assert np.allclose(
        [
            double_inverted.translation.x,
            double_inverted.translation.y,
            double_inverted.translation.z,
        ],
        [
            simple_transform.translation.x,
            simple_transform.translation.y,
            simple_transform.translation.z,
        ],
        atol=1e-6,
    )
    assert np.allclose(
        [
            double_inverted.rotation.x,
            double_inverted.rotation.y,
            double_inverted.rotation.z,
            double_inverted.rotation.w,
        ],
        [
            simple_transform.rotation.x,
            simple_transform.rotation.y,
            simple_transform.rotation.z,
            simple_transform.rotation.w,
        ],
        atol=1e-6,
    )


def testInvertTransformRandom(another_transform):
    inverted = TransformUtils.invertTransform(another_transform)
    combined = TransformUtils.combineTransforms(another_transform, inverted)
    identity = TransformUtils.invertTransform(combined)
    assert np.allclose(
        [identity.translation.x, identity.translation.y, identity.translation.z],
        [0, 0, 0],
        atol=1e-6,
    )
    assert np.allclose(
        [
            identity.rotation.x,
            identity.rotation.y,
            identity.rotation.z,
            identity.rotation.w,
        ],
        [0, 0, 0, 1],
        atol=1e-6,
    )


def testCombineTransformsIdentity(simple_transform, identity_transform):
    combined = TransformUtils.combineTransforms(simple_transform, identity_transform)
    assert np.allclose(
        [combined.translation.x, combined.translation.y, combined.translation.z],
        [
            simple_transform.translation.x,
            simple_transform.translation.y,
            simple_transform.translation.z,
        ],
        atol=1e-6,
    )


def testCombineTransformsReverse(simple_transform):
    inverted = TransformUtils.invertTransform(simple_transform)
    combined = TransformUtils.combineTransforms(simple_transform, inverted)
    assert np.allclose(
        [combined.translation.x, combined.translation.y, combined.translation.z],
        [0, 0, 0],
        atol=1e-6,
    )
    assert np.allclose(
        [
            combined.rotation.x,
            combined.rotation.y,
            combined.rotation.z,
            combined.rotation.w,
        ],
        [0, 0, 0, 1],
        atol=1e-6,
    )


def testCombineTransformsTwoNonIdentity(simple_transform, another_transform):
    combined = TransformUtils.combineTransforms(simple_transform, another_transform)
    assert isinstance(combined, Transform)
    assert not np.allclose(
        [combined.translation.x, combined.translation.y, combined.translation.z],
        [0, 0, 0],
    )


def testTransformPoseIdentity(simple_pose, identity_transform):
    transformed_pose = TransformUtils.transformPoseToParentFrame(
        simple_pose, identity_transform
    )
    assert np.allclose(
        [
            transformed_pose.position.x,
            transformed_pose.position.y,
            transformed_pose.position.z,
        ],
        [simple_pose.position.x, simple_pose.position.y, simple_pose.position.z],
        atol=1e-6,
    )


def testTransformPoseSimple(simple_pose, simple_transform):
    transformed_pose = TransformUtils.transformPoseToParentFrame(
        simple_pose, simple_transform
    )
    assert isinstance(transformed_pose, Pose)
    assert not np.allclose(
        [
            transformed_pose.position.x,
            transformed_pose.position.y,
            transformed_pose.position.z,
        ],
        [simple_pose.position.x, simple_pose.position.y, simple_pose.position.z],
    )


def testTransformPoseReverse(simple_pose, simple_transform):
    forward_pose = TransformUtils.transformPoseToParentFrame(
        simple_pose, simple_transform
    )
    inverted_transform = TransformUtils.invertTransform(simple_transform)
    backward_pose = TransformUtils.transformPoseToParentFrame(
        forward_pose, inverted_transform
    )
    assert np.allclose(
        [backward_pose.position.x, backward_pose.position.y, backward_pose.position.z],
        [simple_pose.position.x, simple_pose.position.y, simple_pose.position.z],
        atol=1e-5,
    )


def testPoseToMatrixAndBack(simple_pose):
    matrix = TransformUtils.poseToMatrix(simple_pose)
    pose_reconstructed = TransformUtils.matrixToPose(matrix)
    assert np.allclose(
        [
            pose_reconstructed.position.x,
            pose_reconstructed.position.y,
            pose_reconstructed.position.z,
        ],
        [simple_pose.position.x, simple_pose.position.y, simple_pose.position.z],
        atol=1e-6,
    )
    assert np.allclose(
        [
            pose_reconstructed.orientation.x,
            pose_reconstructed.orientation.y,
            pose_reconstructed.orientation.z,
            pose_reconstructed.orientation.w,
        ],
        [
            simple_pose.orientation.x,
            simple_pose.orientation.y,
            simple_pose.orientation.z,
            simple_pose.orientation.w,
        ],
        atol=1e-6,
    )


def testTransformToMatrixAndBack(simple_transform):
    matrix = TransformUtils.transformToMatrix(simple_transform)
    transform_reconstructed = TransformUtils.matrixToTransform(matrix)
    assert np.allclose(
        [
            transform_reconstructed.translation.x,
            transform_reconstructed.translation.y,
            transform_reconstructed.translation.z,
        ],
        [
            simple_transform.translation.x,
            simple_transform.translation.y,
            simple_transform.translation.z,
        ],
        atol=1e-6,
    )
    assert np.allclose(
        [
            transform_reconstructed.rotation.x,
            transform_reconstructed.rotation.y,
            transform_reconstructed.rotation.z,
            transform_reconstructed.rotation.w,
        ],
        [
            simple_transform.rotation.x,
            simple_transform.rotation.y,
            simple_transform.rotation.z,
            simple_transform.rotation.w,
        ],
        atol=1e-6,
    )
