import uuid
import time
import threading
from typing import Dict, Optional, Any


class SessionManager:
    def __init__(self, max_sessions: int, timeout: int, cleanup_interval: int = 300):
        self._sessions: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._max_sessions = max_sessions
        self._timeout = timeout
        self._cleanup_interval = cleanup_interval
        self._cleanup_thread: Optional[threading.Thread] = None
        self._running = False

    def create_session(self, scene_name: str) -> Optional[str]:
        with self._lock:
            if len(self._sessions) >= self._max_sessions:
                return None

            token = str(uuid.uuid4())
            self._sessions[token] = {
                "scene_name": scene_name,
                "scene": None,
                "robots": [],
                "objects": [],
                "camera": None,
                "recording": False,
                "last_activity": time.time(),
            }
            return token

    def get_session(self, token: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._sessions.get(token)

    def update_activity(self, token: str) -> None:
        with self._lock:
            if token in self._sessions:
                self._sessions[token]["last_activity"] = time.time()

    def destroy_session(self, token: str) -> bool:
        with self._lock:
            if token in self._sessions:
                del self._sessions[token]
                return True
            return False

    def get_session_count(self) -> int:
        with self._lock:
            return len(self._sessions)

    def cleanup_expired(self) -> int:
        now = time.time()
        expired = []
        with self._lock:
            for token, session in self._sessions.items():
                if now - session["last_activity"] > self._timeout:
                    expired.append(token)
            for token in expired:
                del self._sessions[token]
        return len(expired)

    def start_cleanup_thread(self) -> None:
        if self._cleanup_thread is not None:
            return
        self._running = True
        self._cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
        self._cleanup_thread.start()

    def stop_cleanup_thread(self) -> None:
        self._running = False
        if self._cleanup_thread:
            self._cleanup_thread.join(timeout=1)
            self._cleanup_thread = None

    def _cleanup_loop(self) -> None:
        while self._running:
            time.sleep(self._cleanup_interval)
            self.cleanup_expired()
