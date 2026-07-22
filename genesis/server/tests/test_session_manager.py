import pytest
import time
from genesis_server.session_manager import SessionManager


def test_create_session_returns_token():
    manager = SessionManager(max_sessions=10, timeout=3600)
    token = manager.create_session("empty")
    assert token is not None
    assert isinstance(token, str)
    assert len(token) == 36  # UUID format


def test_get_session_returns_session_data():
    manager = SessionManager(max_sessions=10, timeout=3600)
    token = manager.create_session("empty")
    session = manager.get_session(token)
    assert session is not None
    assert session["scene_name"] == "empty"
    assert "last_activity" in session


def test_get_session_invalid_token_returns_none():
    manager = SessionManager(max_sessions=10, timeout=3600)
    session = manager.get_session("invalid-token")
    assert session is None


def test_destroy_session():
    manager = SessionManager(max_sessions=10, timeout=3600)
    token = manager.create_session("empty")
    assert manager.destroy_session(token) is True
    assert manager.get_session(token) is None


def test_max_sessions_limit():
    manager = SessionManager(max_sessions=2, timeout=3600)
    token1 = manager.create_session("empty")
    token2 = manager.create_session("empty")
    token3 = manager.create_session("empty")
    assert token1 is not None
    assert token2 is not None
    assert token3 is None  # Should fail, at capacity


def test_session_count():
    manager = SessionManager(max_sessions=10, timeout=3600)
    assert manager.get_session_count() == 0
    manager.create_session("empty")
    assert manager.get_session_count() == 1
    manager.create_session("empty")
    assert manager.get_session_count() == 2


def test_update_activity():
    manager = SessionManager(max_sessions=10, timeout=3600)
    token = manager.create_session("empty")
    session = manager.get_session(token)
    old_activity = session["last_activity"]
    time.sleep(0.1)
    manager.update_activity(token)
    session = manager.get_session(token)
    assert session["last_activity"] > old_activity


def test_cleanup_expired():
    manager = SessionManager(max_sessions=10, timeout=0)  # 0 second timeout
    token = manager.create_session("empty")
    time.sleep(0.1)
    expired_count = manager.cleanup_expired()
    assert expired_count == 1
    assert manager.get_session(token) is None
