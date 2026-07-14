import pytest
import json
import threading
import time
import sys
from http.client import HTTPConnection
from unittest.mock import patch, Mock, MagicMock


# Track used ports to avoid conflicts
_port_counter = [19900]


def get_free_port():
    """Get a free port for testing."""
    _port_counter[0] += 1
    return _port_counter[0]


@pytest.fixture
def mock_genesis():
    """Mock genesis module for all tests."""
    mock_gs = MagicMock()
    mock_gs.init = Mock()
    mock_gs.Scene = Mock(return_value=MagicMock())
    mock_gs.amdgpu = "amdgpu"
    mock_gs.options.ViewerOptions = Mock()
    mock_gs.options.SimOptions = Mock()
    mock_gs.morphs.Plane = Mock()
    mock_gs.morphs.MJCF = Mock()

    with patch.dict(sys.modules, {"genesis": mock_gs}):
        # Clear cached imports
        for mod in list(sys.modules.keys()):
            if mod.startswith("genesis_server.") and mod != "genesis_server.config":
                del sys.modules[mod]
        yield mock_gs


def test_create_env_returns_token(mock_genesis):
    from genesis_server.server import GenesisServer

    port = get_free_port()
    server = GenesisServer(port=port)
    server.allow_reuse_address = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    try:
        conn = HTTPConnection("localhost", port)
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "create_env", "params": {"scene": "empty"}}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert response.status == 200
        assert "token" in data
        assert data["status"] == "ok"
    finally:
        server.shutdown()


def test_invalid_action_returns_error(mock_genesis):
    from genesis_server.server import GenesisServer

    port = get_free_port()
    server = GenesisServer(port=port)
    server.allow_reuse_address = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    try:
        conn = HTTPConnection("localhost", port)
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "unknown_action", "token": "abc"}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert data["status"] == "error"
        assert "Unknown action" in data["message"] or "Invalid" in data["message"]
    finally:
        server.shutdown()


def test_missing_token_returns_error(mock_genesis):
    from genesis_server.server import GenesisServer

    port = get_free_port()
    server = GenesisServer(port=port)
    server.allow_reuse_address = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    try:
        conn = HTTPConnection("localhost", port)
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "step", "params": {"steps": 10}}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert data["status"] == "error"
        assert "token" in data["message"].lower() or "session" in data["message"].lower()
    finally:
        server.shutdown()


def test_step_with_valid_token(mock_genesis):
    from genesis_server.server import GenesisServer

    port = get_free_port()
    server = GenesisServer(port=port)
    server.allow_reuse_address = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    try:
        conn = HTTPConnection("localhost", port)

        # Create environment first
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "create_env", "params": {"scene": "empty"}}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())
        token = data["token"]

        # Now step with the token
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "step", "token": token, "params": {"steps": 10}}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert data["status"] == "ok"
    finally:
        server.shutdown()


def test_destroy_env(mock_genesis):
    from genesis_server.server import GenesisServer

    port = get_free_port()
    server = GenesisServer(port=port)
    server.allow_reuse_address = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    try:
        conn = HTTPConnection("localhost", port)

        # Create environment
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "create_env", "params": {"scene": "empty"}}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())
        token = data["token"]

        # Destroy environment
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "destroy_env", "token": token}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert data["status"] == "ok"

        # Try to use destroyed session - should fail
        conn.request(
            "POST", "/",
            body=json.dumps({"action": "step", "token": token, "params": {"steps": 1}}),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert data["status"] == "error"
    finally:
        server.shutdown()


def test_admin_get_status(mock_genesis):
    from genesis_server.server import GenesisServer

    port = get_free_port()
    server = GenesisServer(port=port)
    server.allow_reuse_address = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    try:
        conn = HTTPConnection("localhost", port)
        conn.request(
            "POST", "/",
            body=json.dumps({
                "action": "admin_get_status",
                "params": {"password": "admin123"}
            }),
            headers={"Content-Type": "application/json"}
        )
        response = conn.getresponse()
        data = json.loads(response.read())

        assert data["status"] == "ok"
        assert "sessions" in data
    finally:
        server.shutdown()
