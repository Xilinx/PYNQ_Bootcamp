import pytest
from unittest.mock import Mock, patch, MagicMock
import sys


def test_simulation_init():
    # Patch genesis at the module level
    mock_gs = MagicMock()
    with patch.dict(sys.modules, {"genesis": mock_gs}):
        # Force reimport
        if "genesis_server.simulation" in sys.modules:
            del sys.modules["genesis_server.simulation"]
        from genesis_server.simulation import GenesisSimulation

        sim = GenesisSimulation("empty")
        assert sim.scene_name == "empty"
        assert sim._initialized is False


def test_simulation_build():
    mock_gs = MagicMock()
    mock_scene = MagicMock()
    mock_robot = MagicMock()
    mock_scene.add_entity = Mock(return_value=mock_robot)
    mock_scene.add_camera = Mock(return_value=Mock())
    mock_gs.Scene = Mock(return_value=mock_scene)
    mock_gs.amdgpu = "amdgpu"
    mock_gs.options.ViewerOptions = Mock()
    mock_gs.options.SimOptions = Mock()
    mock_gs.morphs.Plane = Mock()
    mock_gs.morphs.MJCF = Mock()

    with patch.dict(sys.modules, {"genesis": mock_gs}):
        if "genesis_server.simulation" in sys.modules:
            del sys.modules["genesis_server.simulation"]
        from genesis_server.simulation import GenesisSimulation

        sim = GenesisSimulation("empty")
        sim.build()

        assert sim._initialized is True
        mock_gs.init.assert_called_once()
        mock_scene.build.assert_called_once()


def test_add_cube():
    mock_gs = MagicMock()
    mock_scene = MagicMock()
    mock_entity = MagicMock()
    mock_scene.add_entity = Mock(return_value=mock_entity)
    mock_scene.add_camera = Mock(return_value=Mock())
    mock_gs.Scene = Mock(return_value=mock_scene)
    mock_gs.amdgpu = "amdgpu"
    mock_gs.options.ViewerOptions = Mock()
    mock_gs.options.SimOptions = Mock()
    mock_gs.morphs.Plane = Mock()
    mock_gs.morphs.MJCF = Mock()
    mock_gs.morphs.Box = Mock()

    with patch.dict(sys.modules, {"genesis": mock_gs}):
        if "genesis_server.simulation" in sys.modules:
            del sys.modules["genesis_server.simulation"]
        from genesis_server.simulation import GenesisSimulation

        sim = GenesisSimulation("empty")
        sim.build()

        obj_id = sim.add_object("cube", [0.5, 0, 0.02], size=[0.04, 0.04, 0.04])

        assert obj_id == 0
        assert len(sim.objects) == 1
        assert sim.objects[0]["type"] == "cube"


def test_add_sphere():
    mock_gs = MagicMock()
    mock_scene = MagicMock()
    mock_entity = MagicMock()
    mock_scene.add_entity = Mock(return_value=mock_entity)
    mock_scene.add_camera = Mock(return_value=Mock())
    mock_gs.Scene = Mock(return_value=mock_scene)
    mock_gs.amdgpu = "amdgpu"
    mock_gs.options.ViewerOptions = Mock()
    mock_gs.options.SimOptions = Mock()
    mock_gs.morphs.Plane = Mock()
    mock_gs.morphs.MJCF = Mock()
    mock_gs.morphs.Sphere = Mock()

    with patch.dict(sys.modules, {"genesis": mock_gs}):
        if "genesis_server.simulation" in sys.modules:
            del sys.modules["genesis_server.simulation"]
        from genesis_server.simulation import GenesisSimulation

        sim = GenesisSimulation("empty")
        sim.build()

        obj_id = sim.add_object("sphere", [0.3, 0.1, 0.05], radius=0.03)

        assert obj_id == 0
        assert len(sim.objects) == 1
        assert sim.objects[0]["type"] == "sphere"


def test_step():
    mock_gs = MagicMock()
    mock_scene = MagicMock()
    mock_scene.add_entity = Mock(return_value=MagicMock())
    mock_scene.add_camera = Mock(return_value=Mock())
    mock_gs.Scene = Mock(return_value=mock_scene)
    mock_gs.amdgpu = "amdgpu"
    mock_gs.options.ViewerOptions = Mock()
    mock_gs.options.SimOptions = Mock()
    mock_gs.morphs.Plane = Mock()
    mock_gs.morphs.MJCF = Mock()

    with patch.dict(sys.modules, {"genesis": mock_gs}):
        if "genesis_server.simulation" in sys.modules:
            del sys.modules["genesis_server.simulation"]
        from genesis_server.simulation import GenesisSimulation

        sim = GenesisSimulation("empty")
        sim.build()

        sim.step(100)

        assert mock_scene.step.call_count == 100


def test_reset():
    mock_gs = MagicMock()
    mock_scene = MagicMock()
    mock_scene.add_entity = Mock(return_value=MagicMock())
    mock_scene.add_camera = Mock(return_value=Mock())
    mock_gs.Scene = Mock(return_value=mock_scene)
    mock_gs.amdgpu = "amdgpu"
    mock_gs.options.ViewerOptions = Mock()
    mock_gs.options.SimOptions = Mock()
    mock_gs.morphs.Plane = Mock()
    mock_gs.morphs.MJCF = Mock()

    with patch.dict(sys.modules, {"genesis": mock_gs}):
        if "genesis_server.simulation" in sys.modules:
            del sys.modules["genesis_server.simulation"]
        from genesis_server.simulation import GenesisSimulation

        sim = GenesisSimulation("empty")
        sim.build()

        sim.reset()

        mock_scene.reset.assert_called_once()
