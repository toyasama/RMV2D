import os
import yaml
import pytest

from rmv_library.parameters.yaml_handler import YamlHandler
from rmv_library.parameters.frame_parameter import FramesParameters
from rmv_library.parameters.visualization_parameter import VisualizationParameters

TEST_FILE_PATH = "test_rmv.yaml"


@pytest.fixture
def setupTestFile():
    """Fixture to create and delete the test YAML file."""
    with open(TEST_FILE_PATH, "w", encoding="utf-8") as f:
        yaml.dump({}, f)

    yield

    if os.path.exists(TEST_FILE_PATH):
        os.remove(TEST_FILE_PATH)


def testLoadYamlFileExists(setupTestFile):
    """Test loading a YAML file that exists."""
    handler = YamlHandler(TEST_FILE_PATH)
    data = handler.loadYaml()
    assert isinstance(data, dict)
    assert "RMV" in data
    assert "visualizations" in data["RMV"]
    assert "frames" in data["RMV"]


def testLoadYamlFileNotExists():
    """Test that loading a non-existent YAML file raises FileNotFoundError."""
    if os.path.exists(TEST_FILE_PATH):
        os.remove(TEST_FILE_PATH)

    handler = YamlHandler(TEST_FILE_PATH)
    with pytest.raises(FileNotFoundError):
        handler.loadYaml()


def testSaveAndLoadYaml(setupTestFile):
    frames = FramesParameters()
    visualization = VisualizationParameters()

    frames.main_frame = "world"
    visualization.width = 1280
    visualization.height = 720

    handler = YamlHandler(TEST_FILE_PATH)
    handler.save(frames, visualization)

    handler_reload = YamlHandler(TEST_FILE_PATH)
    data = handler_reload.loadYaml()

    assert data["RMV"]["frames"]["main_frame"] == "world"
    assert data["RMV"]["visualizations"]["width"] == 1280
    assert data["RMV"]["visualizations"]["height"] == 720


def testConvertBooleanStringToBool():
    handler = YamlHandler(TEST_FILE_PATH)
    assert handler._convertData("true") is True
    assert handler._convertData("false") is False
    assert handler._convertData("something else") == "something else"


def testSaveYamlHandlesErrorsGracefully(monkeypatch, setupTestFile):
    frames = FramesParameters()
    visualization = VisualizationParameters()
    handler = YamlHandler(TEST_FILE_PATH)

    def raiseError(*args, **kwargs):
        raise IOError("Mock IOError")

    monkeypatch.setattr("builtins.open", raiseError)

    handler.save(frames, visualization)
